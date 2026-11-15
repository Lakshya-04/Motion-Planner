#!/usr/bin/env python3
"""Records a run driven by something else (the manipulation manager) as a score_run CSV.

Subscribes like contact_test.py --record (tool point in base_footprint from TF, ground-truth force) and writes
PATH and PATH.wall.json when it receives SIGINT/SIGTERM or after --timeout seconds. The wall is the smoke wall:
the plane x = 0.98 in base_footprint, y 0..0.3, z 1.0..1.3 (override with the options).
"""
import argparse
import os
import signal
import sys
import threading
import time

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from contact_test import ContactTest  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--record', required=True)
    ap.add_argument('--timeout', type=float, default=900.0)
    ap.add_argument('--wall-x', type=float, default=0.98)
    ap.add_argument('--wall-y', type=float, nargs=2, default=[0.0, 0.3])
    ap.add_argument('--wall-z', type=float, nargs=2, default=[1.0, 1.3])
    args = ap.parse_args()
    rclpy.init()
    node = ContactTest()
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    t0 = time.monotonic()
    while not stop.is_set() and time.monotonic() - t0 < args.timeout:
        time.sleep(0.1)
    n = node.write_recording(args.record, args.wall_x, tuple(args.wall_y), tuple(args.wall_z))
    print(f'recorded {n} samples to {args.record}', flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
