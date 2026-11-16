#!/usr/bin/env python3
# imu-log.py — 在 PC 上实时接收手机端 IMUOnly.dylib 的日志
#
# 手机端需要设置环境变量/重签时注入 TL_LOG_HOST=192.168.10.2:9804
# （没设置也没关系，日志会写到 App 沙盒 tmp/tl_imu.log，可用爱思助手导出）
#
# 用法：python imu-log.py [--port 9804]

import argparse
import socket
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=9804)
    args = ap.parse_args()

    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", args.port))
    srv.listen(4)
    print(f"[log] listening {args.port}", flush=True)

    while True:
        c, addr = srv.accept()
        print(f"[log] phone connected {addr[0]}", flush=True)
        c.settimeout(None)
        buf = b""
        try:
            while True:
                d = c.recv(4096)
                if not d:
                    break
                buf += d
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    ts = time.strftime("%H:%M:%S")
                    print(f"{ts} {line.decode('utf-8', 'replace')}", flush=True)
        except Exception as e:
            print(f"[log] error {e}", flush=True)
        finally:
            c.close()
            print("[log] phone disconnected", flush=True)


if __name__ == "__main__":
    main()
