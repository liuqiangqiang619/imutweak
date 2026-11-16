#!/usr/bin/env python3
# imu-sender.py — 把 IMU 样本发给手机端的 IMUOnly.dylib
#
# 用法：
#   python imu-sender.py --usbmux                 # USB（推荐）
#   python imu-sender.py --host 192.168.10.5      # Wi-Fi
#   python imu-sender.py --usbmux --mode tianle   # 从天乐（本地 9803）读角速度
#   python imu-sender.py --usbmux --mode sine     # 正弦测试（默认）
#
# 协议（与 IMUOnly.m 对应）：
#   12 字节头：u8 type=6, u8 flags, u16 len(BE), u32 seq(BE), u32 pts_ms(BE)
#   负载：n 个样本，每样本 3 个 little-endian float32（gx, gy, gz，rad/s），100Hz

import argparse
import math
import socket
import struct
import sys
import time
import os

USBMUX_ADDR = ("127.0.0.1", 27015)
RATE = 100.0
BATCH = 5              # 每包 5 个样本 = 50ms
HDR = struct.Struct(">BBHII")
SAMPLE = struct.Struct("<fff")


def usbmux_connect(device_port, udid=None):
    import plistlib

    def send(sock, obj):
        body = plistlib.dumps(obj)
        sock.sendall(struct.pack("<IIII", 16 + len(body), 1, 8, 1) + body)

    def recv(sock):
        hdr = b""
        while len(hdr) < 16:
            c = sock.recv(16 - len(hdr))
            if not c:
                return None
            hdr += c
        total = struct.unpack("<I", hdr[:4])[0]
        rest = b""
        while len(rest) < total - 16:
            c = sock.recv(total - 16 - len(rest))
            if not c:
                return None
            rest += c
        try:
            return plistlib.loads(rest)
        except Exception:
            return None

    d = socket.create_connection(USBMUX_ADDR, timeout=5)
    send(d, {"MessageType": "ListDevices", "ClientVersionString": "imu-sender",
             "ProgName": "imu-sender", "kLibUSBMuxVersion": 3})
    r = recv(d) or {}
    devs = r.get("DeviceList", [])
    if not devs:
        d.close()
        raise RuntimeError("usbmuxd 没有发现设备（数据线接了吗？）")
    dev = None
    for x in devs:
        if udid and x.get("Properties", {}).get("SerialNumber") != udid:
            continue
        dev = x
        break
    if dev is None:
        d.close()
        raise RuntimeError("没找到指定 udid 的设备")
    dev_id = dev.get("DeviceID")
    d.close()

    c = socket.create_connection(USBMUX_ADDR, timeout=5)
    send(c, {"MessageType": "Connect", "ClientVersionString": "imu-sender",
             "ProgName": "imu-sender", "kLibUSBMuxVersion": 3,
             "DeviceID": dev_id, "PortNumber": device_port & 0xFFFF})
    r = recv(c) or {}
    if r.get("MessageType") != "Result" or r.get("Number") != 0:
        num = r.get("Number")
        c.close()
        raise RuntimeError(f"连接端口 {device_port} 失败，Number={num}")
    c.settimeout(None)
    return c


def connect_sink(args):
    """持续尝试，直到连上手机端监听端口"""
    while True:
        try:
            if args.usbmux:
                s = usbmux_connect(args.port, args.udid)
            else:
                s = socket.create_connection((args.host, args.port), timeout=5)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"[imu] connected {'usb' if args.usbmux else args.host}", flush=True)
            return s
        except Exception as e:
            print(f"[imu] wait for phone: {e}", flush=True)
            time.sleep(2)


def gen_sine(t):
    """测试信号：三轴不同频率、幅度 ~0.35 rad/s"""
    return (0.30 * math.sin(2 * math.pi * 0.17 * t),
            0.22 * math.sin(2 * math.pi * 0.11 * t + 1.1),
            0.10 * math.sin(2 * math.pi * 0.07 * t + 2.0))


def gen_from_tianle(sock):
    """从天乐读取：12 字节头 + n*12 字节 float 样本（与手机同一格式）"""
    hdr = b""
    while len(hdr) < 12:
        c = sock.recv(12 - len(hdr))
        if not c:
            raise RuntimeError("tianle closed")
        hdr += c
    _, _, ln, _, _ = HDR.unpack(hdr)
    body = b""
    while len(body) < ln:
        c = sock.recv(ln - len(body))
        if not c:
            raise RuntimeError("tianle closed")
        body += c
    return [SAMPLE.unpack_from(body, i)[:3] for i in range(0, ln, 12)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.10.5")
    ap.add_argument("--port", type=int, default=9802)
    ap.add_argument("--usbmux", action="store_true")
    ap.add_argument("--udid", default=None)
    ap.add_argument("--mode", choices=["sine", "tianle"], default="sine")
    ap.add_argument("--tianle", default="127.0.0.1:9803")
    args = ap.parse_args()

    src = None
    if args.mode == "tianle":
        h, p = args.tianle.split(":")
        while True:
            try:
                src = socket.create_connection((h, int(p)), timeout=5)
                print(f"[imu] tianle {args.tianle} connected", flush=True)
                break
            except Exception as e:
                print(f"[imu] wait for tianle: {e}", flush=True)
                time.sleep(1)

    sink = connect_sink(args)
    seq = 0
    tick = 1.0 / RATE
    next_t = time.perf_counter()
    t0 = time.perf_counter()

    try:
        while True:
            if src is not None:
                try:
                    batch = gen_from_tianle(src)
                except Exception as e:
                    print(f"[imu] tianle lost: {e}", flush=True)
                    src = None
                    continue
            else:
                batch = []
                now = time.perf_counter()
                for i in range(BATCH):
                    batch.append(gen_sine(now - t0 - (BATCH - 1 - i) * tick))

            payload = b"".join(SAMPLE.pack(*s) for s in batch)
            hdr = HDR.pack(6, 0, len(payload), seq & 0xFFFFFFFF,
                           int((time.perf_counter() - t0) * 1000) & 0xFFFFFFFF)
            try:
                sink.sendall(hdr + payload)
            except Exception as e:
                print(f"[imu] send failed: {e}", flush=True)
                try:
                    sink.close()
                except Exception:
                    pass
                sink = connect_sink(args)
            seq += 1

            next_t += tick * len(batch)
            dt = next_t - time.perf_counter()
            if dt > 0:
                time.sleep(dt)
            elif dt < -1.0:
                next_t = time.perf_counter()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            sink.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
