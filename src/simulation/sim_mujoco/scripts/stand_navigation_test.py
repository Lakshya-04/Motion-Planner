#!/usr/bin/env python3
"""Stand navigation check for the MuJoCo backend (run_stand_navigation.sh starts everything):

  1. drive to a staging pose off the wall through `navigate_to_stand` (a first use of the action);
  2. call reachability `~/plan_stands` for the smoke wall's coverage poses with the base profile and
     the staging pose as the start; the wall is at x = 0.98 m in the odom frame (the base starts at the
     origin, facing it);
  3. drive the arm base to the first stand and check the arrival error against /odom and TF, that the
     footprint is clear of the wall, that the arm and lift joints held their pose, and that the raw F/T
     wrench stayed within 2 N of its resting reading.

Exit code 0 on success.
"""
import argparse
import math
import os
import sys
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from reachability_msgs.srv import PlanStands
from sensor_msgs.msg import JointState
from stand_navigator.core import arm_to_footprint, load_profile
from stand_navigator_msgs.action import NavigateToStand
from tf2_ros import Buffer, TransformListener

WALL_X = 0.98


def yaw_of(q):
    return 2.0 * math.atan2(q.z, q.w)


def wall_targets():
    """Contact-pass poses on the wall (the shape coverage_planner writes: tool axis into the wall)."""
    poses = []
    for y in (-0.25, -0.11, 0.03, 0.17, 0.31, 0.45):
        for z in (1.01, 1.15, 1.29):
            p = Pose()
            p.position.x, p.position.y, p.position.z = WALL_X, y, z
            p.orientation.x = p.orientation.y = p.orientation.z = p.orientation.w = 0.5
            poses.append(p)
    return poses


class T(Node):
    def __init__(self):
        super().__init__('stand_navigation_test')
        self.lock = threading.Lock()
        self.odom, self.js, self.raw = None, None, None
        self.truth = None   # /odom_truth, only there when the sim runs with odom_realism
        self.create_subscription(Odometry, '/odom_truth', lambda m: setattr(self, 'truth', m), 10)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(JointState, '/joint_states', self._js, 10)
        self.create_subscription(WrenchStamped, '/sim_mujoco/ft_raw', self._raw, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToStand, 'navigate_to_stand')
        self.plan = self.create_client(PlanStands, '/reachability_node/plan_stands')
        self.max_dev, self.max_df, self.f0, self.arm0 = 0.0, 0.0, None, None

    def _odom(self, m):
        with self.lock:
            self.odom = m

    def _js(self, m):
        with self.lock:
            self.js = dict(zip(m.name, m.position))

    def _raw(self, m):
        w = m.wrench
        with self.lock:
            self.raw = [w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z]

    def state(self):
        with self.lock:
            return self.odom, self.js, self.raw

    def pose(self):
        o = self.state()[0].pose.pose
        return o.position.x, o.position.y, yaw_of(o.orientation)

    def sample(self):
        _, js, raw = self.state()
        self.max_dev = max(self.max_dev, max(abs(js[k] - v) for k, v in self.arm0.items()))
        self.max_df = max(self.max_df, max(abs(a - b) for a, b in zip(raw, self.f0)))

    def drive(self, arm_x, arm_y, yaw, timeout=120.0):
        goal = NavigateToStand.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'odom'
        goal.pose.pose.position.x, goal.pose.pose.position.y = arm_x, arm_y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        fut = self.nav.send_goal_async(goal)
        t0 = time.monotonic()
        while not fut.done():
            time.sleep(0.05)
        handle = fut.result()
        if not handle.accepted:
            return None
        rf = handle.get_result_async()
        while not rf.done() and time.monotonic() - t0 < timeout:
            self.sample()
            time.sleep(0.05)
        return rf.result().result if rf.done() else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', required=True, help='robot profile YAML (the base: section)')
    ap.add_argument('--timeout', type=float, default=90.0)
    ap.add_argument('--tol-xy', type=float, default=0.03)
    ap.add_argument('--tol-yaw', type=float, default=0.03)
    args = ap.parse_args()
    prof = load_profile(args.profile)
    rclpy.init()
    node = T()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    failures = []

    def check(name, ok, detail):
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}", flush=True)
        if not ok:
            failures.append(name)

    def finish():
        print('STAND NAVIGATION', 'FAILED: ' + ', '.join(failures) if failures else 'PASSED', flush=True)
        os._exit(1 if failures else 0)

    t0 = time.monotonic()
    while time.monotonic() - t0 < args.timeout:
        odom, js, raw = node.state()
        if (odom is not None and js is not None and raw is not None and 'arm_elbow_joint' in js
                and node.nav.server_is_ready() and node.plan.service_is_ready()):
            break
        time.sleep(0.2)
    else:
        odom, js, raw = node.state()
        print(f'[FAIL] startup: odom {odom is not None}, joint_states {js is not None}, ft_raw {raw is not None}, '
              f'navigate_to_stand {node.nav.server_is_ready()}, plan_stands {node.plan.service_is_ready()}', flush=True)
        os._exit(1)
    time.sleep(2.0)
    node.arm0 = {k: v for k, v in node.state()[1].items() if k.startswith(('arm_', 'lift_'))}
    node.f0 = node.state()[2]

    # 1. staging pose: 0.4 m back and 0.3 m to the right of the start, turned 0.3 rad
    stage = (-0.4, -0.3, 0.3)
    r = node.drive(*stage)
    check('staging drive', r is not None and r.error_code == 0 and r.position_error_m < args.tol_xy,
          'no result' if r is None else f'code {r.error_code} {r.error_msg}, error {r.position_error_m * 1000:.1f} mm')
    if r is None:
        finish()
    time.sleep(1.0)
    mount = prof.arm_mount
    fx, fy, fyaw = node.pose()
    c, s = math.cos(fyaw), math.sin(fyaw)
    sx, sy = fx + c * mount[0] - s * mount[1], fy + s * mount[0] + c * mount[1]

    # 2. plan stands for the wall from the staging pose
    req = PlanStands.Request()
    req.targets_world = wall_targets()
    req.wall_point.x = WALL_X
    req.wall_normal.x = -1.0
    req.lift_min, req.lift_max, req.lift_step = 0.0, 1.5, 0.1
    req.arm_mount_offset_z = 1.17      # lift_mount 0.35 + column 0.8 + mount 0.02 (armBaseHeightAtLiftZero)
    req.has_start, req.start_x, req.start_y, req.start_yaw = True, sx, sy, fyaw
    fut = node.plan.call_async(req)
    t1 = time.monotonic()
    while not fut.done() and time.monotonic() - t1 < 60.0:
        time.sleep(0.05)
    res = fut.result() if fut.done() else None
    check('plan_stands', res is not None and res.success and len(res.stands) >= 1,
          'no reply' if res is None else
          f'{len(res.stands)} stand(s), unreachable {list(res.unreachable)}, {res.message!r}')
    if res is None or not res.stands:
        finish()
    st = res.stands[0]
    print(f'first stand: arm base ({st.x:.3f}, {st.y:.3f}) yaw {st.yaw:.3f}, standoff {st.standoff:.3f}, '
          f'footprint clearance {st.footprint_clearance:.3f}, covers {len(st.target_indices)}/{len(req.targets_world)}',
          flush=True)
    fpx, fpy, _ = arm_to_footprint(st.x, st.y, st.yaw, prof.arm_mount)
    print(f'  base_footprint goal ({fpx:.3f}, {fpy:.3f}); distance from staging pose '
          f'{math.hypot(fpx - fx, fpy - fy):.2f} m', flush=True)

    # 3. drive there
    t_drive = time.monotonic()
    r = node.drive(st.x, st.y, st.yaw)
    took = time.monotonic() - t_drive
    check('navigate_to_stand result', r is not None and r.error_code == 0,
          'no result' if r is None else f'code {r.error_code} {r.error_msg}, took {took:.1f} s')
    if r is None:
        finish()
    time.sleep(1.5)
    x1, y1, yaw1 = node.pose()
    c, s = math.cos(yaw1), math.sin(yaw1)
    armx, army = x1 + c * mount[0] - s * mount[1], y1 + s * mount[0] + c * mount[1]
    err = math.hypot(armx - st.x, army - st.y)
    yerr = abs(math.atan2(math.sin(yaw1 - st.yaw), math.cos(yaw1 - st.yaw)))
    check('arm base arrival error (odom)', err < args.tol_xy and yerr < args.tol_yaw,
          f'{err * 1000:.1f} mm, {yerr:.4f} rad (limits {args.tol_xy * 1000:.0f} mm, {args.tol_yaw} rad)')
    if node.truth is not None:
        o = node.truth.pose.pose
        xt, yt, yawt = o.position.x, o.position.y, yaw_of(o.orientation)
        ct, stt = math.cos(yawt), math.sin(yawt)
        errt = math.hypot(xt + ct * mount[0] - stt * mount[1] - st.x, yt + stt * mount[0] + ct * mount[1] - st.y)
        yerrt = abs(math.atan2(math.sin(yawt - st.yaw), math.cos(yawt - st.yaw)))
        check('arm base arrival error (odom_truth)', errt < args.tol_xy and yerrt < args.tol_yaw,
              f'{errt * 1000:.1f} mm, {yerrt:.4f} rad (limits {args.tol_xy * 1000:.0f} mm, {args.tol_yaw} rad); '
              f'reported odom was off the truth by {math.hypot(x1 - xt, y1 - yt) * 1000:.1f} mm, '
              f'{math.degrees(math.atan2(math.sin(yaw1 - yawt), math.cos(yaw1 - yawt))):+.2f} deg')
    try:
        tf = node.tf_buffer.lookup_transform('odom', 'arm_base_link', rclpy.time.Time())
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        check('TF odom -> arm_base_link at the stand', math.hypot(tx - st.x, ty - st.y) < args.tol_xy,
              f'tf ({tx:.3f}, {ty:.3f}) vs stand ({st.x:.3f}, {st.y:.3f})')
    except Exception as e:  # tf2 raises several exception types
        check('TF odom -> arm_base_link', False, repr(e))
    # the footprint's nearest point to the wall plane (polygon from the profile), along the wall normal
    with open(args.profile) as f:
        fp = yaml.safe_load(f)['base']['footprint']
    gap = min(WALL_X - (x1 + c * px - s * py) for px, py in fp)
    check('footprint clear of the wall', gap > 0.0,
          f'gap {gap:.3f} m (plan said {st.footprint_clearance:.3f} m)')
    node.sample()
    check('arm and lift joints hold their pose', node.max_dev < 0.02, f'max deviation {node.max_dev:.4f} rad/m')
    raw = node.state()[2]
    check('F/T stays sane', node.max_df < 2.0 and all(abs(v) < 100.0 for v in raw),
          f'max |F/T - resting| = {node.max_df:.2f}, raw now {[round(v, 2) for v in raw]}')
    xa, ya, _ = node.pose()
    time.sleep(2.0)
    xb, yb, _ = node.pose()
    check('base holds at the stand', math.hypot(xb - xa, yb - ya) < 0.005,
          f'drift {math.hypot(xb - xa, yb - ya) * 1000:.2f} mm in 2 s')
    finish()


if __name__ == '__main__':
    main()
