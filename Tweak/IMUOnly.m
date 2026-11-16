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
#import <string.h>
#import <stdio.h>
#import <unistd.h>
#import <dispatch/dispatch.h>
#import <stdlib.h>

static void TLLogWrite(const char* line);

#define TL_LOG(fmt, ...) do { \
    NSString* _m = [NSString stringWithFormat:(fmt), ##__VA_ARGS__]; \
    NSLog(@"[TLIMU] %@", _m); \
    TLLogWrite(_m.UTF8String); \
} while (0)

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
static NSTimer* g_logTimer = nil;
static BOOL g_kvcOk = NO;
static BOOL g_haveData = NO;

static double g_yaw = 0, g_pitch = 0, g_roll = 0;   // 由角速度积分出的姿态
static double g_biasX = 0, g_biasY = 0, g_biasZ = 0;


// ------------------------------------------------------------------ 日志：NSLog + 文件 + 可选 TCP 回传
static FILE* g_logf = NULL;
static int   g_logSock = -1;

static void TLLogWrite(const char* line) {
    if (g_logf) { fprintf(g_logf, "%s\n", line); fflush(g_logf); }
    if (g_logSock >= 0) {
        char buf[1024];
        int n = snprintf(buf, sizeof(buf), "%s\n", line);
        if (n > 0) {
            if (send(g_logSock, buf, (size_t)n, 0) <= 0) {
                close(g_logSock);
                g_logSock = -1;
            }
        }
    }
}

static void TLLogInit(void) {
    NSString* path = [NSTemporaryDirectory() stringByAppendingPathComponent:@"tl_imu.log"];
    g_logf = fopen(path.UTF8String, "a");
    const char* hp = getenv("TL_LOG_HOST");      // 形如 192.168.10.2:9804
    if (hp && *hp) {
        char host[128] = {0};
        int port = 9804;
        const char* colon = strchr(hp, ':');
        if (colon) {
            size_t n = (size_t)(colon - hp); if (n > 127) n = 127;
            memcpy(host, hp, n);
            port = atoi(colon + 1);
        } else {
            strncpy(host, hp, sizeof(host) - 1);
        }
        int s = socket(AF_INET, SOCK_STREAM, 0);
        struct sockaddr_in sa; memset(&sa, 0, sizeof(sa));
        sa.sin_family = AF_INET;
        sa.sin_port = htons((uint16_t)port);
        if (s >= 0 && inet_pton(AF_INET, host, &sa.sin_addr) == 1 &&
            connect(s, (struct sockaddr*)&sa, sizeof(sa)) == 0) {
            int on = 1;
            setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
            g_logSock = s;
        } else if (s >= 0) {
            close(s);
        }
    }
}


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
    if (ls < 0) { TL_LOG(@"socket failed"); return NULL; }
    int on = 1;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_addr.s_addr = htonl(INADDR_ANY);
    sa.sin_port = htons(TLIMU_PORT);
    if (bind(ls, (struct sockaddr*)&sa, sizeof(sa)) < 0) {
        TL_LOG(@"bind %d failed", TLIMU_PORT);
        close(ls);
        return NULL;
    }
    listen(ls, 2);
    TL_LOG(@"listening %d", TLIMU_PORT);

    for (;;) {
        int cs = accept(ls, NULL, NULL);
        if (cs < 0) { usleep(200000); continue; }
        setsockopt(cs, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
        TL_LOG(@"sender connected");
        uint8_t hdr[12];
        while (ReadFull(cs, hdr, sizeof(hdr))) {
            uint8_t type = hdr[0];
            uint16_t len = BE16(hdr + 2);
            if (type != 6 || len == 0 || len > TLIMU_MAX_BATCH * 12) {
                TL_LOG(@"bad frame type=%u len=%u", type, len);
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
            if (!g_haveData) { g_haveData = YES; TL_LOG(@"first batch n=%d", n); }
        }
        TL_LOG(@"sender disconnected (samples=%llu)", g_rxSamples);
        close(cs);
    }
    return NULL;
}


// ------------------------------------------------------------------ 字段写入
// CMDeviceMotion 的 rotationRate / userAcceleration / gravity 是 C 结构体，KVC 未必能写；
// 优先直接按 ivar 偏移写内存，失败再退回 KVC。
static BOOL g_ivarOk = NO;
static BOOL g_dumped = NO;

static void DumpIvars(void) {
    unsigned int cnt = 0;
    Ivar* ivars = class_copyIvarList(NSClassFromString(@"CMDeviceMotion"), &cnt);
    for (unsigned int i = 0; i < cnt; ++i) {
        TL_LOG(@"ivar %s : %s", ivar_getName(ivars[i]), ivar_getTypeEncoding(ivars[i]));
    }
    if (ivars) free(ivars);
}

// 严格校验：只写「确实存在、类型对得上、且不越过实例大小」的 ivar。
// 之前无条件按 ddd 写 24 字节，一旦结构体是 fff 或 ivar 布局不同，就会写坏内存导致闪退。
static BOOL IvAddr(id obj, const char* name, size_t need, const char* encHint, void** out) {
    if (!obj || !name) return NO;
    Class cls = object_getClass(obj);
    Ivar iv = class_getInstanceVariable(cls, name);
    if (!iv) return NO;
    const char* t = ivar_getTypeEncoding(iv);
    ptrdiff_t off = ivar_getOffset(iv);
    size_t isz = class_getInstanceSize(cls);
    if (!t || off < 0 || (size_t)off + need > isz) return NO;
    if (encHint && !strstr(t, encHint)) return NO;
    *out = (char*)(__bridge void*)obj + off;
    return YES;
}

static BOOL WriteStruct3(id obj, const char* name, const double v[3]) {
    void* p = NULL;
    if (!IvAddr(obj, name, 24, "ddd", &p)) return NO;
    memcpy(p, v, 24);
    return YES;
}

static BOOL WriteDouble(id obj, const char* name, double v) {
    void* p = NULL;
    if (!IvAddr(obj, name, 8, "d", &p)) return NO;
    memcpy(p, &v, 8);
    return YES;
}

// ------------------------------------------------------------------ 用 getter 覆盖（不伪造对象）
// 教训：iOS 的 CMDeviceMotion 只有一个 ivar `_internal`（CMDeviceMotionInternal），
// [[CMDeviceMotion alloc] init] 出来的对象 _internal 是 nil，抖音一读 rotationRate/attitude 就崩。
// 所以让真实对象继续流动，只把 rotationRate / timestamp / gravity 三个 getter 换成我们的实现。

static CMRotationRate (*orig_rotationRate)(id, SEL) = NULL;
static NSTimeInterval (*orig_timestamp)(id, SEL) = NULL;
static CMAcceleration (*orig_gravity)(id, SEL) = NULL;

static double g_iYaw = 0, g_iPitch = 0, g_iRoll = 0, g_iLast = 0;

static void TLIntegrate(double t, const float g[3]) {
    double dt = g_iLast > 0 ? (t - g_iLast) : 0.01;
    if (dt <= 0 || dt > 0.2) dt = 0.01;
    g_iLast = t;
    g_iYaw   += g[1] * dt;
    g_iPitch += g[0] * dt;
    g_iRoll  += g[2] * dt;
}

static CMRotationRate tl_rotationRate(id self, SEL _cmd) {
    // 超过 500ms 没有新样本就退回真实传感器：手机静止时读数为 0，
    // 比冻结在最后一个值更像真机（冻结值 = 明显破绽）
    BOOL stale = (g_lastRxTime <= 0) || (CACurrentMediaTime() - g_lastRxTime > 0.5);
    if (!g_haveData || stale) return orig_rotationRate ? orig_rotationRate(self, _cmd) : (CMRotationRate){0, 0, 0};
    double t = CACurrentMediaTime();
    float g[3] = {0, 0, 0};
    SampleAt(t - g_delayComp, g);
    TLIntegrate(t, g);
    const double nz = 0.0035;
    double nx = ((double)arc4random() / UINT32_MAX - 0.5) * nz;
    double ny = ((double)arc4random() / UINT32_MAX - 0.5) * nz;
    double nzz = ((double)arc4random() / UINT32_MAX - 0.5) * nz;
    return (CMRotationRate){ g[0] + nx, g[1] + ny, g[2] + nzz };
}

static NSTimeInterval tl_timestamp(id self, SEL _cmd) {
    // 与 CMSampleBuffer 的 PTS 同一根表（mach_absolute_time）
    return CACurrentMediaTime();
}

static CMAcceleration tl_gravity(id self, SEL _cmd) {
    BOOL stale = (g_lastRxTime <= 0) || (CACurrentMediaTime() - g_lastRxTime > 0.5);
    if (!g_haveData || stale) return orig_gravity ? orig_gravity(self, _cmd) : (CMAcceleration){0, 0, -1};
    double gx = -sin(g_iPitch);
    double gy =  sin(g_iRoll) * cos(g_iPitch);
    double gz =  cos(g_iRoll) * cos(g_iPitch);
    return (CMAcceleration){ gx, gy, gz };
}

// 只用来周期性打点，不参与数据通路
static void TickLog(void) {
    if (!g_haveData) return;
    double now = CACurrentMediaTime();
    float g[3] = {0, 0, 0};
    SampleAt(now - g_delayComp, g);
    TL_LOG(@"wx=%.3f wy=%.3f wz=%.3f rx=%llu age=%.0fms", g[0], g[1], g[2],
           g_rxSamples, g_lastRxTime > 0 ? (now - g_lastRxTime) * 1000.0 : -1);
}

// ------------------------------------------------------------------ hook
static void (*orig_start_q)(id, SEL, NSOperationQueue*, id) = NULL;
static void (*orig_start_rf)(id, SEL, NSInteger, NSOperationQueue*, id) = NULL;
static void (*orig_start_plain)(id, SEL) = NULL;

static void (^g_userHandler)(CMDeviceMotion*, NSError*) = nil;

static void my_start_q(id self, SEL _cmd, NSOperationQueue* q, id handler) {
    g_userHandler = [handler copy];
    if (!orig_start_q) return;
    orig_start_q(self, _cmd, q, ^(CMDeviceMotion* m, NSError* e) {
        void (^h)(CMDeviceMotion*, NSError*) = g_userHandler;
        if (h) h(m, e);
    });
}

static void my_start_rf(id self, SEL _cmd, NSInteger frame, NSOperationQueue* q, id handler) {
    g_userHandler = [handler copy];
    if (!orig_start_rf) return;
    orig_start_rf(self, _cmd, frame, q, ^(CMDeviceMotion* m, NSError* e) {
        void (^h)(CMDeviceMotion*, NSError*) = g_userHandler;
        if (h) h(m, e);
    });
}

static void my_start_plain(id self, SEL _cmd) {
    if (orig_start_plain) orig_start_plain(self, _cmd);
}

static void HookInstance(Class c, SEL s, void* impl) {
    Method m = class_getInstanceMethod(c, s);
    if (m) method_setImplementation(m, (IMP)impl);
}

static void TLInstallHooks(void) {
    @try {
        Class c = NSClassFromString(@"CMMotionManager");
        if (!c) { TL_LOG(@"CMMotionManager not found"); return; }
        Method mq = class_getInstanceMethod(c, @selector(startDeviceMotionUpdatesToQueue:withHandler:));
        if (mq) { orig_start_q = (void*)method_getImplementation(mq); method_setImplementation(mq, (IMP)my_start_q); }
        Method mr = class_getInstanceMethod(c, @selector(startDeviceMotionUpdatesUsingReferenceFrame:toQueue:withHandler:));
        if (mr) { orig_start_rf = (void*)method_getImplementation(mr); method_setImplementation(mr, (IMP)my_start_rf); }
        Method mp = class_getInstanceMethod(c, @selector(startDeviceMotionUpdates));
        if (mp) { orig_start_plain = (void*)method_getImplementation(mp); method_setImplementation(mp, (IMP)my_start_plain); }
        // 不再伪造 isDeviceMotionAvailable / isGyroAvailable（真机本来就有，少改一处少一分风险）

        Class dm = NSClassFromString(@"CMDeviceMotion");
        if (dm) {
            Method mr = class_getInstanceMethod(dm, @selector(rotationRate));
            if (mr) { orig_rotationRate = (void*)method_getImplementation(mr); method_setImplementation(mr, (IMP)tl_rotationRate); }
            Method mt = class_getInstanceMethod(dm, @selector(timestamp));
            if (mt) { orig_timestamp = (void*)method_getImplementation(mt); method_setImplementation(mt, (IMP)tl_timestamp); }
            Method mg = class_getInstanceMethod(dm, @selector(gravity));
            if (mg) { orig_gravity = (void*)method_getImplementation(mg); method_setImplementation(mg, (IMP)tl_gravity); }
            TL_LOG(@"motion getters hooked (rate=%d ts=%d grav=%d)", orig_rotationRate != NULL, orig_timestamp != NULL, orig_gravity != NULL);
        }

        g_logTimer = [NSTimer scheduledTimerWithTimeInterval:5.0 repeats:YES block:^(NSTimer* t){ TickLog(); }];
        TL_LOG(@"hooks installed");
    } @catch (NSException* e) {
        TL_LOG(@"hook install failed: %@", e.reason);
    }
}

__attribute__((constructor))
static void TLIMUInit(void) {
    TLLogInit();
    const char* d = getenv("TL_IMU_DELAY");
    if (d) { double v = atof(d); if (v >= 0 && v < 0.5) g_delayComp = v; }

    pthread_t th;
    pthread_create(&th, NULL, ImuRxThread, NULL);

    if (getenv("TL_IMU_EARLY")) {
        TLInstallHooks();
        TL_LOG(@"init done (early), delayComp=%.0fms", g_delayComp * 1000.0);
    } else {
        // 与 Cam.dylib 这类插件共存：等它把相机/音频 hook 装完再挂，避开启动期的初始化竞争
        dispatch_after(dispatch_time(DISPATCH_TIME_NOW, (int64_t)(1.5 * NSEC_PER_SEC)),
                       dispatch_get_main_queue(), ^{
            TLInstallHooks();
            TL_LOG(@"init done (deferred 1.5s), delayComp=%.0fms", g_delayComp * 1000.0);
        });
    }
}
