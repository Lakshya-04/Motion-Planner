#!/usr/bin/env python3
"""Plans stands for a coverage plan and drives the arm base to one of them (run_stand_pass.sh starts everything).

  1. reads the coverage plan (coverage_planner's coverage_output.json, poses in the world / odom frame);
  2. calls reachability `plan_stands` for them with the base's current pose as the start;
  3. drives to stand --index through `navigate_to_stand` and prints the arrival.

The plan's wall is the plane x = --wall-x (normal -x). Exit code 0 when the base arrived. The chosen stand and the
base's start and end poses are printed as `key value` lines for the caller.
"""
import argparse
import json
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from reachability_msgs.srv import PlanStands
from stand_navigator.core import load_profile
from stand_navigator_msgs.action import NavigateToStand


def yaw_of(q):
    return 2.0 * math.atan2(q.z, q.w)


def coverage_targets(path):
    with open(path) as f:
        doc = json.load(f)
    out = []
    for p in doc['final_lifted_and_expanded']['path']['poses']:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (p['position'][k] for k in 'xyz')
        o = p['orientation']
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (o[k] for k in 'xyzw')
        out.append(pose)
    return out


class Driver(Node):
    def __init__(self):
        super().__init__('stand_drive')
        self.odom = None
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.nav = ActionClient(self, NavigateToStand, 'navigate_to_stand')
        self.plan = self.create_client(PlanStands, '/reachability_node/plan_stands')

    def _odom(self, m):
        self.odom = m

    def base_pose(self):
        o = self.odom.pose.pose
        return o.position.x, o.position.y, yaw_of(o.orientation)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--coverage', required=True)
    ap.add_argument('--profile', required=True)
    ap.add_argument('--wall-x', type=float, required=True)
    ap.add_argument('--index', type=int, default=0)
    ap.add_argument('--timeout', type=float, default=120.0)
    args = ap.parse_args()
    prof = load_profile(args.profile)
    rclpy.init()
    node = Driver()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 90.0:
        if node.odom is not None and node.nav.server_is_ready() and node.plan.service_is_ready():
            break
        time.sleep(0.2)
    else:
        print('stand_drive: odom, navigate_to_stand or plan_stands not up')
        os._exit(1)
    time.sleep(1.0)
    x0, y0, yaw0 = node.base_pose()
    mx, my = prof.arm_mount
    sx = x0 + math.cos(yaw0) * mx - math.sin(yaw0) * my
    sy = y0 + math.sin(yaw0) * mx + math.cos(yaw0) * my
    print(f'start_base {x0:.4f} {y0:.4f} {yaw0:.4f}')

    req = PlanStands.Request()
    req.targets_world = coverage_targets(args.coverage)
    req.wall_point.x = args.wall_x
    req.wall_normal.x = -1.0
    req.lift_min, req.lift_max, req.lift_step = 0.0, 1.5, 0.1
    req.arm_mount_offset_z = 1.17
    req.has_start, req.start_x, req.start_y, req.start_yaw = True, sx, sy, yaw0
    fut = node.plan.call_async(req)
    t1 = time.monotonic()
    while not fut.done() and time.monotonic() - t1 < 60.0:
        time.sleep(0.05)
    res = fut.result() if fut.done() else None
    if res is None or not res.success or len(res.stands) <= args.index:
        print('stand_drive: plan_stands failed:', None if res is None else (res.message, list(res.unreachable)))
        os._exit(2)
    for i, st in enumerate(res.stands):
        print(f'stand {i}: arm base ({st.x:.3f}, {st.y:.3f}) yaw {st.yaw:.3f} covers {list(st.target_indices)}')
    st = res.stands[args.index]
    print(f'stand_distance_m {math.hypot(st.x - sx, st.y - sy):.3f}')

    goal = NavigateToStand.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'odom'
    goal.pose.pose.position.x, goal.pose.pose.position.y = st.x, st.y
    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(st.yaw / 2), math.cos(st.yaw / 2)
    gf = node.nav.send_goal_async(goal)
    while not gf.done():
        time.sleep(0.05)
    handle = gf.result()
    if not handle.accepted:
        print('stand_drive: goal rejected')
        os._exit(3)
    rf = handle.get_result_async()
    t2 = time.monotonic()
    while not rf.done() and time.monotonic() - t2 < args.timeout:
        time.sleep(0.05)
    if not rf.done():
        print('stand_drive: drive timed out')
        os._exit(4)
    r = rf.result().result
    time.sleep(1.5)
    x1, y1, yaw1 = node.base_pose()
    print(f'end_base {x1:.4f} {y1:.4f} {yaw1:.4f}')
    print(f'arrival error {r.position_error_m * 1000:.1f} mm, code {r.error_code} {r.error_msg}')
    os._exit(0 if r.error_code == 0 else 5)


if __name__ == '__main__':
    main()
