#!/usr/bin/env python3
"""Send a goal across the empty world past a box, through Nav2 or the stand_navigator Nav2 adapter (run_nav2_goal.sh).

The base starts at the origin facing +x with a 0.9 x 0.6 m footprint; the box (centre x 2.0, 0.3 x 0.6 m) is on the
straight line to the goal. Reports arrival error, the closest the footprint came to the box and how far it swung out.
Exit code 0 when it arrived within tolerance and the footprint never touched the box.
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

BOX = (2.0, 0.0, 0.15, 0.30)    # cx, cy, half x, half y
FOOT = (0.45, 0.30)             # half x, half y of the footprint


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def corners(cx, cy, hx, hy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c * x - s * y, cy + s * x + c * y) for x, y in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))]


def seg_dist(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / (ax * ax + ay * ay)))
    return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)


def overlap(p, q):
    for poly in (p, q):
        for i in range(4):
            ex, ey = poly[(i + 1) % 4][0] - poly[i][0], poly[(i + 1) % 4][1] - poly[i][1]
            ax, ay = -ey, ex
            pa = [ax * x + ay * y for x, y in p]
            qa = [ax * x + ay * y for x, y in q]
            if max(pa) < min(qa) or max(qa) < min(pa):
                return False
    return True


def gap(pose):
    """Distance between the footprint and the box in the plane, negative when they overlap."""
    f = corners(pose[0], pose[1], FOOT[0], FOOT[1], pose[2])
    b = corners(BOX[0], BOX[1], BOX[2], BOX[3], 0.0)
    if overlap(f, b):
        return -1.0
    d = min(seg_dist(p, poly[i], poly[(i + 1) % 4]) for p, poly in [(p, b) for p in f] + [(p, f) for p in b]
            for i in range(4))
    return d


class T(Node):
    def __init__(self):
        super().__init__('gz_nav2_goal_test')
        self.odom, self.track = None, []
        self.create_subscription(Odometry, '/odom', self._odom, 10)

    def _odom(self, m):
        self.odom = m
        p = m.pose.pose
        self.track.append((p.position.x, p.position.y, yaw_of(p.orientation), time.monotonic()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=('nav2', 'adapter'), required=True)
    ap.add_argument('--goal', type=float, nargs=3, default=[4.0, 0.0, 0.0], metavar=('X', 'Y', 'YAW'))
    ap.add_argument('--timeout', type=float, default=120.0)
    ap.add_argument('--tol-xy', type=float, default=0.15)
    ap.add_argument('--tol-yaw', type=float, default=0.15)
    args = ap.parse_args()
    rclpy.init()
    node = T()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    if args.mode == 'nav2':
        from nav2_msgs.action import NavigateToPose as A
        client = ActionClient(node, A, 'navigate_to_pose')
    else:
        from stand_navigator_msgs.action import NavigateToStand as A
        client = ActionClient(node, A, 'navigate_to_stand')
    t0 = time.monotonic()
    while time.monotonic() - t0 < 180.0 and not (node.odom is not None and client.server_is_ready()):
        time.sleep(0.2)
    if not client.server_is_ready():
        print('[FAIL] action server not available', flush=True)
        os._exit(1)
    time.sleep(3.0)     # costmaps need a few scans
    goal = A.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'odom'
    goal.pose.pose.position.x, goal.pose.pose.position.y = args.goal[0], args.goal[1]
    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(args.goal[2] / 2), math.cos(args.goal[2] / 2)
    fut = client.send_goal_async(goal)
    while not fut.done():
        time.sleep(0.05)
    handle = fut.result()
    print(f'goal accepted: {handle.accepted}', flush=True)
    if not handle.accepted:
        os._exit(1)
    rf = handle.get_result_async()
    t1 = time.monotonic()
    while not rf.done() and time.monotonic() - t1 < args.timeout:
        time.sleep(0.05)
    done = rf.done()
    if not done:
        handle.cancel_goal_async()
    took = time.monotonic() - t1
    time.sleep(1.5)
    res = rf.result() if done else None
    status = None if res is None else res.status
    code = None if res is None else getattr(res.result, 'error_code', None)
    msg = '' if res is None else getattr(res.result, 'error_msg', '')
    x, y, yaw = node.track[-1][:3]
    pos_err = math.hypot(x - args.goal[0], y - args.goal[1])
    yaw_err = abs(math.atan2(math.sin(yaw - args.goal[2]), math.cos(yaw - args.goal[2])))
    gaps = [gap(p[:3]) for p in node.track if p[3] >= t1 - 0.5]
    ymax = max(abs(p[1]) for p in node.track if p[3] >= t1 - 0.5)
    print(f'result: {"none within timeout" if not done else f"status {status} code {code} {msg}"}; took {took:.1f} s; '
          f'final ({x:.3f}, {y:.3f}, yaw {yaw:.3f}); position error {pos_err:.3f} m, yaw error {yaw_err:.3f} rad; '
          f'min footprint-to-box gap {min(gaps):.3f} m (negative: overlap); max |y| {ymax:.2f} m', flush=True)
    fails = []
    if pos_err > args.tol_xy or yaw_err > args.tol_yaw:
        fails.append('arrival')
    if min(gaps) <= 0.0:
        fails.append('touched the box')
    for f in ('arrival', 'touched the box'):
        print(f'[{"FAIL" if f in fails else "PASS"}] {f}', flush=True)
    print(f'NAV2 GOAL ({args.mode})', 'FAILED: ' + ', '.join(fails) if fails else 'PASSED', flush=True)
    os._exit(1 if fails else 0)


if __name__ == '__main__':
    main()
