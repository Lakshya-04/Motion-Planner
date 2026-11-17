#!/usr/bin/env python3
"""Saves PNG stills from the Gazebo colour camera (`rgb_camera:=true`, topic /rgb_camera/image).

    capture_still.py OUT_DIR [--every S] [--count N] [--timeout S] [--prefix NAME]

Writes OUT_DIR/NAME_0000.png, ... every S seconds of wall time (default 20) until N images (default 1) or --timeout, or
on SIGINT/SIGTERM. rgb8 images only; the PNG is written with zlib, no imaging library needed.
"""
import argparse
import os
import signal
import struct
import threading
import time
import zlib

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def write_png(path, rgb):
    h, w, _ = rgb.shape
    raw = b''.join(b'\x00' + rgb[r].tobytes() for r in range(h))

    def chunk(tag, data):
        c = struct.pack('>I', len(data)) + tag + data
        return c + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff)
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                + chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))


class Grab(Node):
    def __init__(self):
        super().__init__('capture_still')
        self.latest = None
        self.create_subscription(Image, '/rgb_camera/image', self._on, qos_profile_sensor_data)

    def _on(self, m):
        if m.encoding != 'rgb8':
            return
        self.latest = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step // 3, 3)[:, :m.width]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out_dir')
    ap.add_argument('--every', type=float, default=20.0)
    ap.add_argument('--count', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=1800.0)
    ap.add_argument('--prefix', default='still')
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    rclpy.init()
    node = Grab()
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    t0, n, last = time.monotonic(), 0, -1e9
    while not stop.is_set() and n < args.count and time.monotonic() - t0 < args.timeout:
        if node.latest is not None and time.monotonic() - last >= args.every:
            path = os.path.join(args.out_dir, f'{args.prefix}_{n:04d}.png')
            write_png(path, node.latest)
            print(f'capture_still: {path} {node.latest.shape[1]}x{node.latest.shape[0]}', flush=True)
            n, last = n + 1, time.monotonic()
        time.sleep(0.1)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
