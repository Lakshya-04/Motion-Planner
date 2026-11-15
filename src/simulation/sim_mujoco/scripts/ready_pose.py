#!/usr/bin/env python3
"""Moves the arm to the smoke test's FREE pose (tool pointing at the wall, a short way off it) through the
controller's action and prints the tool pose reached in base_footprint: `x y z qx qy qz qw`. The manager pass
starts from here instead of the stowed pose, as the scripted wall pass does."""
import os
import sys
import threading
import time

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from smoke_test import FREE  # noqa: E402
from wall_pass_test import WallPass  # noqa: E402


def main():
    rclpy.init()
    node = WallPass()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 60:
        js, _, _ = node.state()
        if js is not None and node.client.server_is_ready():
            break
        time.sleep(0.2)
    else:
        print('ready_pose: no joint states / action server', file=sys.stderr)
        os._exit(1)
    code = node.move(FREE, 8.0)
    node.hold(1.0)
    p = node.tcp_pose()
    t, r = p.translation, p.rotation
    print(f'ready_pose: error_code {code} tcp {t.x:.4f} {t.y:.4f} {t.z:.4f} q {r.x:.4f} {r.y:.4f} {r.z:.4f} {r.w:.4f}',
          flush=True)
    os._exit(0 if code == 0 else 2)


if __name__ == '__main__':
    main()
