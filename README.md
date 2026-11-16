# IMUOnly — 独立陀螺仪注入插件

给抖音补上「画面在动、陀螺仪也在动」的一致性。**只 hook CoreMotion，不碰画面和音频**，因此可以和 `Cam.dylib` 这类画音注入插件同时注入同一个 App。

## 它做什么

```
天乐(PC) 从画面运动算角速度 ──USB/Wi-Fi──► IMUOnly.dylib ──► CMMotionManager 回调
                                                   │
                                                   └─ 时间戳锚定 CACurrentMediaTime()（与相机 PTS 同源）
```

- 手机端监听 `9802`，PC 主动连（USB 走 usbmux，Wi-Fi 直连也行）
- 收到样本后按到达时间回推铺开（每样本 10ms），播放时再减 60ms 延迟补偿
- 每个 `CMMotionManager` 回调返回一个伪造的 `CMDeviceMotion`：真实 `rotationRate`、积分出的 `gravity`、`timestamp` 用宿主时钟
- 没收到数据时输出 0（手机静止），并叠加传感器噪声与缓慢零偏漂移

## 协议

```
12 字节头（网络序）: u8 type=6 | u8 flags | u16 len | u32 seq | u32 pts_ms
负载: n = len/12 个样本，每样本 3 个 little-endian float32 (gx, gy, gz) rad/s，固定 100Hz
```

## 构建

Windows 上编不了 arm64 iOS dylib，走 GitHub Actions：

```bash
git init && git add . && git commit -m "imu-only tweak"
git remote add origin https://github.com/<user>/<repo>.git
git push -u origin main
```

CI 用 `theos/sdks` 的 iPhoneOS SDK + clang/lld 交叉编译，产物 `IMUOnly.dylib`（约 20–40KB）。

## 注入

1. 解密抖音 IPA
2. 全能签里把 `IMUOnly.dylib` 加到注入列表（**可以和 Cam.dylib 同时注入**）
3. 个人证书重签安装

## 联调

PC 侧：

```bash
# 纯正弦测试（先验证手机端是否在替换陀螺仪）
python imu-sender.py --usbmux

# 接天乐（天乐把角速度发到本地 9803）
python imu-sender.py --usbmux --mode tianle
```

手机端看日志（Console.app 搜 `TLIMU`）：

```
[TLIMU] listening 9802
[TLIMU] sender connected
[TLIMU] first batch n=5
[TLIMU] motion KVC ok
[TLIMU] wx=0.30 wy=0.22 wz=0.10 rx=12345 age=48ms
```

抖音里转动手机 / 播放会动的视频时，陀螺仪读数应跟随画面变化，而不是恒 0。

## 注意

- `TL_IMU_DELAY` 环境变量可调延迟补偿（默认 0.06 秒）
- 如果 `KVC ok` 没打印，说明该 iOS 版本 `CMDeviceMotion` 的私有 ivar 名变了，需要按 `[TLIMU] ivar ...` 日志（可在构造里打开 `DumpIvars`）调整键名
- 本插件只补 IMU；画面/音频的注入由另一个插件负责，两者互不干扰

## 天乐已内置发送端

`tl_push.exe` 启动即监听 **9803**（`TL_IMU=0` 可关，`TL_IMU_PORT` 可改），每 50ms 发一包 5 个样本，
样本来自画面运动（`UpdateVideoMotion` 的 mvx/mvy + 扰动旋转），角速度已做低频平滑。

所以正常情况下只要跑：

```bash
python imu-sender.py --usbmux --mode tianle
```

实测：`imu listen on 9803` → sender 转发 → 手机端约 **96.7Hz**（100Hz 标称，受调度抖动影响）。
