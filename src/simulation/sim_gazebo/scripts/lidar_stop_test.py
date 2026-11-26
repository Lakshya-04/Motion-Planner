#!/usr/bin/env python3
"""Obstacle stop from the simulated lidar, Gazebo (run_lidar_stop.sh starts everything).

The base (base_footprint at the origin, facing +x, footprint 0.9 x 0.6 m) is sent 3 m ahead; a box with its front face at
x = 1.45 m is in the way. `--mode with`: the navigator reads /scan and must end with the typed OBSTACLE result, stopped
`--stop-m` or more in front of the box; `--mode without`: it cannot see the box and the base ends against it.
Exit code 0 when the expectation of the mode holds.
"""
import argparse
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from stand_navigator_msgs.action import NavigateToStand

BOX_FACE_X = 1.45
FRONT_X = 0.45      # front edge of the stock footprint in base_footprint; the lidar sits there


class T(Node):
    def __init__(self):
        super().__init__('gz_lidar_stop_test')
        self.odom, self.scans, self.scan = None, 0, None
        self.track = []
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        self.nav = ActionClient(self, NavigateToStand, 'navigate_to_stand')

    def _odom(self, m):
        self.odom = m
        self.track.append(m.pose.pose.position.x)

    def _scan(self, m):
        self.scan, self.scans = m, self.scans + 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=('with', 'without'), required=True)
    ap.add_argument('--stop-m', type=float, default=0.30)
    ap.add_argument('--timeout', type=float, default=120.0)
    args = ap.parse_args()
    rclpy.init()
    node = T()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    lidar = args.mode == 'with'
    failures = []

    def check(name, ok, detail):
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}", flush=True)
        if not ok:
            failures.append(name)

    def finish():
        print(f'GAZEBO LIDAR STOP ({args.mode} lidar)', 'FAILED: ' + ', '.join(failures) if failures else 'PASSED', flush=True)
        os._exit(1 if failures else 0)

    t0 = time.monotonic()
    while time.monotonic() - t0 < args.timeout:
        if node.odom is not None and node.nav.server_is_ready() and (node.scans > 0 or not lidar):
            break
        time.sleep(0.2)
    else:
        print(f'[FAIL] startup: odom {node.odom is not None}, navigate_to_stand {node.nav.server_is_ready()}, '
              f'scans {node.scans}', flush=True)
        os._exit(1)
    time.sleep(2.0)
    if lidar:
        s = node.scan
        k = round(-s.angle_min / s.angle_increment)
        got = s.ranges[k]
        # the lidar is on the front edge of the base, so the range straight ahead is the gap to the box face
        check('scan sees the box', math.isfinite(got) and abs(got - (BOX_FACE_X - FRONT_X)) < 0.05,
              f'ahead {got:.3f} m, geometry {BOX_FACE_X - FRONT_X:.3f} m; {len(s.ranges)} rays, frame '
              f'{s.header.frame_id}, finite rays {sum(math.isfinite(r) for r in s.ranges)}')
    goal = NavigateToStand.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'odom'
    goal.pose.pose.position.x = 3.0
    goal.pose.pose.orientation.w = 1.0
    fut = node.nav.send_goal_async(goal)
    while not fut.done():
        time.sleep(0.05)
    rf = fut.result().get_result_async()
    t1 = time.monotonic()
    while not rf.done() and time.monotonic() - t1 < 60.0:
        time.sleep(0.05)
    r = rf.result().result if rf.done() else None
    time.sleep(1.5)
    x = node.odom.pose.pose.position.x
    gap = BOX_FACE_X - (x + FRONT_X)
    code = None if r is None else r.error_code
    print(f'drive result: {"none within 60 s" if r is None else f"code {r.error_code} {r.error_msg}"}; took '
          f'{time.monotonic() - t1:.1f} s; base x {x:.3f} m; front-edge gap to the box {gap:.3f} m '
          f'(negative: overlap), furthest x {max(node.track):.3f}', flush=True)
    if lidar:
        check('typed OBSTACLE result', code == NavigateToStand.Result.OBSTACLE,
              f'code {code} (OBSTACLE is {NavigateToStand.Result.OBSTACLE})')
        check('stops short of the configured stop distance', gap >= args.stop_m - 0.05,
              f'gap {gap:.3f} m, obstacle_stop_m {args.stop_m:.2f} m (0.05 m allowed for scan age)')
    else:
        check('no OBSTACLE result (the stack cannot see the box)', code != NavigateToStand.Result.OBSTACLE, f'code {code}')
        reach = BOX_FACE_X - (max(node.track) + FRONT_X)
        check('base reached the box', reach <= 0.01,
              f'front edge came within {reach:.3f} m of the face (it is pushed back afterwards, final gap {gap:.3f} m)')
    finish()


if __name__ == '__main__':
    main()
