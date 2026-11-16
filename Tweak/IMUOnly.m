// IMUOnly.m — 独立 IMU 注入插件（只管陀螺仪，不碰画面/音频）
//
// 与 Cam.dylib 这类「画面/音频注入」插件共存：
//   它 hook  AVCaptureVideoDataOutput / AVCaptureAudioDataOutput
//   我们 hook CMMotionManager 的 startDeviceMotionUpdates*
//   两者互不冲突。
//
// 数据来源：PC（天乐）从画面运动算出角速度，按 100Hz 打包，
// 通过 TCP 9802 送到手机（手机 listen，PC 主动连；USB 走 usbmux，Wi-Fi 直连亦可）。
//
// 时间基准：
//   CMDeviceMotion.timestamp / CMSampleBuffer PTS 都基于 mach_absolute_time（CACurrentMediaTime），
//   所以这里把样本重新锚定到 CACurrentMediaTime() 上，就与相机同一根表。
//   到达时刻回推 (n-1)*10ms 铺开样本，再减一个可调补偿（默认 60ms）抵消传输延迟。
//
// 协议（12 字节头 + 负载，全部网络字节序）：
//   u8  type   = 6 (IMU)
//   u8  flags  = 0
//   u16 len    = 负载字节数
//   u32 seq
//   u32 pts_ms = PC 侧时间戳（仅调试用，不参与时间基准）
//   负载：n = len/12 个样本，每个 3 个 little-endian float32：gx, gy, gz (rad/s)，固定 100Hz

#import <Foundation/Foundation.h>
#import <CoreMotion/CoreMotion.h>
#import <QuartzCore/QuartzCore.h>
#import <objc/runtime.h>
#import <math.h>
#import <pthread.h>
#import <sys/socket.h>
#import <netinet/in.h>
#import <netinet/tcp.h>
#import <arpa/inet.h>
#import <unistd.h>

#define TLIMU_PORT      9802
#define TLIMU_RATE      100.0
#define TLIMU_RING      2048          // 约 20 秒
#define TLIMU_MAX_BATCH 512

typedef struct { double t; float gx, gy, gz; } ImuSample;

static ImuSample g_ring[TLIMU_RING];
static int g_head = 0;                // 下一个写入位置
static int g_count = 0;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;

static double g_delayComp = 0.060;    // 播放延迟补偿（秒），TL_IMU_DELAY 可改
static double g_lastRxTime = 0;
static unsigned long long g_rxSamples = 0, g_batches = 0;

static id g_handler = nil;
static NSTimer* g_timer = nil;
static BOOL g_kvcOk = NO;
static BOOL g_haveData = NO;

static double g_yaw = 0, g_pitch = 0, g_roll = 0;   // 由角速度积分出的姿态
static double g_biasX = 0, g_biasY = 0, g_biasZ = 0;

// ------------------------------------------------------------------ 工具
static BOOL SetNum(id obj, NSArray<NSString*>* keys, double v) {
    for (NSString* k in keys) {
        @try { [obj setValue:@(v) forKey:k]; return YES; } @catch (NSException* e) {}
    }
    return NO;
}

static BOOL ReadFull(int s, void* buf, size_t n) {
    uint8_t* p = (uint8_t*)buf;
    size_t got = 0;
    while (got < n) {
        ssize_t r = recv(s, p + got, n - got, 0);
        if (r <= 0) return NO;
        got += (size_t)r;
    }
    return YES;
}

static uint16_t BE16(const uint8_t* b) { return (uint16_t)((b[0] << 8) | b[1]); }
static uint32_t BE32(const uint8_t* b) {
    return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
}

// ------------------------------------------------------------------ 样本环
static void PushSamples(const float* data, int n, double endTime) {
    pthread_mutex_lock(&g_lock);
    for (int i = 0; i < n; ++i) {
        ImuSample* s = &g_ring[g_head];
        s->t  = endTime - (double)(n - 1 - i) / TLIMU_RATE;
        s->gx = data[i * 3 + 0];
        s->gy = data[i * 3 + 1];
        s->gz = data[i * 3 + 2];
        g_head = (g_head + 1) % TLIMU_RING;
        if (g_count < TLIMU_RING) g_count++;
    }
    pthread_mutex_unlock(&g_lock);
}

// 在 t 处插值；没有数据返回 NO
static BOOL SampleAt(double t, float* out) {
    BOOL ok = NO;
    pthread_mutex_lock(&g_lock);
    if (g_count >= 2) {
        // 找第一个时间 >= t 的样本（从最旧到最新线性扫，量小）
        int oldest = (g_head - g_count + TLIMU_RING) % TLIMU_RING;
        ImuSample* prev = NULL;
        for (int k = 0; k < g_count; ++k) {
            ImuSample* s = &g_ring[(oldest + k) % TLIMU_RING];
            if (s->t >= t) {
                if (!prev) { out[0] = s->gx; out[1] = s->gy; out[2] = s->gz; }
                else {
                    double span = s->t - prev->t;
                    double a = span > 1e-6 ? (t - prev->t) / span : 1.0;
                    if (a < 0) a = 0; if (a > 1) a = 1;
                    out[0] = (float)(prev->gx + (s->gx - prev->gx) * a);
                    out[1] = (float)(prev->gy + (s->gy - prev->gy) * a);
                    out[2] = (float)(prev->gz + (s->gz - prev->gz) * a);
                }
                ok = YES;
                break;
            }
            prev = s;
        }
        if (!ok && prev) { out[0] = prev->gx; out[1] = prev->gy; out[2] = prev->gz; ok = YES; }
    }
    pthread_mutex_unlock(&g_lock);
    return ok;
}

// ------------------------------------------------------------------ 接收线程
static void* ImuRxThread(void* arg) {
    int ls = socket(AF_INET, SOCK_STREAM, 0);
    if (ls < 0) { NSLog(@"[TLIMU] socket failed"); return NULL; }
    int on = 1;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_addr.s_addr = htonl(INADDR_ANY);
    sa.sin_port = htons(TLIMU_PORT);
    if (bind(ls, (struct sockaddr*)&sa, sizeof(sa)) < 0) {
        NSLog(@"[TLIMU] bind %d failed", TLIMU_PORT);
        close(ls);
        return NULL;
    }
    listen(ls, 2);
    NSLog(@"[TLIMU] listening %d", TLIMU_PORT);

    for (;;) {
        int cs = accept(ls, NULL, NULL);
        if (cs < 0) { usleep(200000); continue; }
        setsockopt(cs, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
        NSLog(@"[TLIMU] sender connected");
        uint8_t hdr[12];
        while (ReadFull(cs, hdr, sizeof(hdr))) {
            uint8_t type = hdr[0];
            uint16_t len = BE16(hdr + 2);
            if (type != 6 || len == 0 || len > TLIMU_MAX_BATCH * 12) {
                NSLog(@"[TLIMU] bad frame type=%u len=%u", type, len);
                break;
            }
            uint8_t payload[TLIMU_MAX_BATCH * 12];
            if (!ReadFull(cs, payload, len)) break;
            int n = len / 12;
            float vals[TLIMU_MAX_BATCH * 3];
            for (int i = 0; i < n * 3; ++i) {
                float f;
                memcpy(&f, payload + i * 4, 4);
                vals[i] = f;
            }
            double now = CACurrentMediaTime();
            PushSamples(vals, n, now);
            g_lastRxTime = now;
            g_rxSamples += n;
            g_batches++;
            if (!g_haveData) { g_haveData = YES; NSLog(@"[TLIMU] first batch n=%d", n); }
        }
        NSLog(@"[TLIMU] sender disconnected (samples=%llu)", g_rxSamples);
        close(cs);
    }
    return NULL;
}

// ------------------------------------------------------------------ 造 CMDeviceMotion
static CMDeviceMotion* MakeMotion(double dt) {
    CMDeviceMotion* m = [[CMDeviceMotion alloc] init];

    float g[3] = {0, 0, 0};
    double now = CACurrentMediaTime();
    BOOL live = SampleAt(now - g_delayComp, g);

    // 没收到数据：全 0（手机静止），加一点传感器噪声
    if (!live) { g[0] = g[1] = g[2] = 0; }

    double nz = 0.0035;
    double gx = g[0] + g_biasX + ((double)arc4random() / UINT32_MAX - 0.5) * nz;
    double gy = g[1] + g_biasY + ((double)arc4random() / UINT32_MAX - 0.5) * nz;
    double gz = g[2] + g_biasZ + ((double)arc4random() / UINT32_MAX - 0.5) * nz;
    g_biasX += ((double)arc4random() / UINT32_MAX - 0.5) * 2e-5;
    g_biasY += ((double)arc4random() / UINT32_MAX - 0.5) * 2e-5;
    g_biasZ += ((double)arc4random() / UINT32_MAX - 0.5) * 2e-5;
    // 慢漂移不无限跑
    if (fabs(g_biasX) > 0.01) g_biasX *= 0.999;
    if (fabs(g_biasY) > 0.01) g_biasY *= 0.999;
    if (fabs(g_biasZ) > 0.01) g_biasZ *= 0.999;

    g_yaw   += gy * dt;
    g_pitch += gx * dt;
    g_roll  += gz * dt;

    double gravX = -sin(g_pitch);
    double gravY =  sin(g_roll) * cos(g_pitch);
    double gravZ =  cos(g_roll) * cos(g_pitch);

    @try {
        BOOL ok = YES;
        ok &= SetNum(m, @[@"_rotationRateX", @"_gyroX", @"rotationRate.x"], gx);
        ok &= SetNum(m, @[@"_rotationRateY", @"_gyroY", @"rotationRate.y"], gy);
        ok &= SetNum(m, @[@"_rotationRateZ", @"_gyroZ", @"rotationRate.z"], gz);
        ok &= SetNum(m, @[@"_userAccelerationX", @"_accelX"], 0.0);
        ok &= SetNum(m, @[@"_userAccelerationY", @"_accelY"], 0.0);
        ok &= SetNum(m, @[@"_userAccelerationZ", @"_accelZ"], 0.0);
        ok &= SetNum(m, @[@"_gravityX", @"gravity.x"], gravX * 9.81);
        ok &= SetNum(m, @[@"_gravityY", @"gravity.y"], gravY * 9.81);
        ok &= SetNum(m, @[@"_gravityZ", @"gravity.z"], gravZ * 9.81);
        ok &= SetNum(m, @[@"_timestamp", @"timestamp"], now);
        if (!g_kvcOk && ok) { g_kvcOk = YES; NSLog(@"[TLIMU] motion KVC ok"); }
    } @catch (NSException* e) {
        NSLog(@"[TLIMU] kvc failed: %@", e.reason);
    }
    return m;
}

static void Tick(void) {
    static double last = 0;
    static unsigned long long n = 0;
    double now = CACurrentMediaTime();
    double dt = last > 0 ? (now - last) : 0.01;
    last = now;
    CMDeviceMotion* m = MakeMotion(dt);
    if (g_handler) {
        void (^h)(CMDeviceMotion*, NSError*) = g_handler;
        h(m, nil);
    }
    if ((n % 500) == 0) {
        float g[3] = {0, 0, 0};
        SampleAt(now - g_delayComp, g);
        NSLog(@"[TLIMU] wx=%.3f wy=%.3f wz=%.3f rx=%llu age=%.0fms", g[0], g[1], g[2],
              g_rxSamples, g_lastRxTime > 0 ? (now - g_lastRxTime) * 1000.0 : -1);
    }
    n++;
}

// ------------------------------------------------------------------ hook
static void (*orig_start_q)(id, SEL, NSOperationQueue*, id) = NULL;
static void (*orig_start_rf)(id, SEL, NSInteger, NSOperationQueue*, id) = NULL;
static void (*orig_start_plain)(id, SEL) = NULL;

static void StartTickerIfNeeded(void) {
    if (g_timer) return;
    dispatch_async(dispatch_get_main_queue(), ^{
        g_timer = [NSTimer scheduledTimerWithTimeInterval:0.01 repeats:YES block:^(NSTimer* t){ Tick(); }];
        [[NSRunLoop mainRunLoop] addTimer:g_timer forMode:NSRunLoopCommonModes];
        NSLog(@"[TLIMU] ticker started");
    });
}

static void my_start_q(id self, SEL _cmd, NSOperationQueue* q, id handler) {
    g_handler = [handler copy];
    StartTickerIfNeeded();
}

static void my_start_rf(id self, SEL _cmd, NSInteger frame, NSOperationQueue* q, id handler) {
    g_handler = [handler copy];
    StartTickerIfNeeded();
}

static void my_start_plain(id self, SEL _cmd) {
    StartTickerIfNeeded();
}

static void HookInstance(Class c, SEL s, void* impl) {
    Method m = class_getInstanceMethod(c, s);
    if (m) method_setImplementation(m, (IMP)impl);
}

__attribute__((constructor))
static void TLIMUInit(void) {
    const char* d = getenv("TL_IMU_DELAY");
    if (d) { double v = atof(d); if (v >= 0 && v < 0.5) g_delayComp = v; }

    Class c = NSClassFromString(@"CMMotionManager");
    if (c) {
        orig_start_q = (void*)method_getImplementation(class_getInstanceMethod(c, @selector(startDeviceMotionUpdatesToQueue:withHandler:)));
        HookInstance(c, @selector(startDeviceMotionUpdatesToQueue:withHandler:), (void*)my_start_q);
        HookInstance(c, @selector(startDeviceMotionUpdatesUsingReferenceFrame:toQueue:withHandler:), (void*)my_start_rf);
        HookInstance(c, @selector(startDeviceMotionUpdates), (void*)my_start_plain);

        Method av = class_getInstanceMethod(c, @selector(isDeviceMotionAvailable));
        if (av) method_setImplementation(av, imp_implementationWithBlock(^BOOL(id _s){ return YES; }));
        Method gv = class_getInstanceMethod(c, @selector(isGyroAvailable));
        if (gv) method_setImplementation(gv, imp_implementationWithBlock(^BOOL(id _s){ return YES; }));
    }

    pthread_t th;
    pthread_create(&th, NULL, ImuRxThread, NULL);
    NSLog(@"[TLIMU] init done, delayComp=%.0fms", g_delayComp * 1000.0);
}
