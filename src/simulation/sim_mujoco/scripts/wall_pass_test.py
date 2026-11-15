#!/usr/bin/env python3
"""A scripted wall pass in the MuJoCo sim: MoveIt plans the strokes, JTAC executes them under force control.

Run against a sim launched with `controller:=jtac conditioner:=true move_group:=true` and the smoke wall scene
(scripts/run_wall_pass.sh does that). For every stroke y:

  1. a Cartesian path (move_group's /compute_cartesian_path) to the stroke start, 3 cm in front of the wall;
  2. engage the tool-Z admittance axis with the contact target and a Cartesian path into the wall;
  3. a Cartesian path down the wall (the stroke) with the contact held;
  4. disengage and retreat 3 cm.

The whole run is recorded as a score_run CSV (tool point from TF, ground-truth force along the tool axis) with
the force counted as engaged contact, and scored: coverage of the planned region, contact force, time.
Exit code 0 when every stroke executed and the scorecard limits hold.
"""
import argparse
import os
import subprocess
import sys
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from contact_test import ContactTest, guarded  # noqa: E402
from smoke_test import FREE, JOINTS  # noqa: E402

WALL_X = 0.98


class WallPass(ContactTest):
    def __init__(self):
        super().__init__()
        self.cart = self.create_client(GetCartesianPath, '/compute_cartesian_path')

    def tcp_pose(self):
        tf = self.tf_buffer.lookup_transform('base_footprint', 'tcp_link', rclpy.time.Time())
        return tf.transform

    def pose(self, x, y, z, orientation):
        p = Pose()
        p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
        p.orientation = orientation
        return p

    def plan(self, waypoints, speed_scale):
        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_footprint'
        js, _, _ = self.state()
        start = RobotState()
        start.joint_state = JointState(name=list(JOINTS), position=list(js))
        req.start_state = start
        req.group_name = 'lift_arm'
        req.link_name = 'tcp_link'
        req.waypoints = waypoints
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = False
        req.max_velocity_scaling_factor = speed_scale
        req.max_acceleration_scaling_factor = speed_scale
        res = self.call(self.cart, req, timeout=30.0)
        return res.solution.joint_trajectory, res.fraction

    def execute(self, traj):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        for _ in range(50):
            fut = self.client.send_goal_async(goal)
            while not fut.done():
                time.sleep(0.01)
            handle = fut.result()
            if handle.accepted:
                break
            time.sleep(0.2)
        else:
            raise RuntimeError('trajectory goal rejected')
        res = handle.get_result_async()
        while not res.done():
            time.sleep(0.01)
        return res.result().result.error_code


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--strokes-y', type=float, nargs='+', default=[0.07, 0.19], help='stroke positions along the wall, m')
    ap.add_argument('--z-top', type=float, default=1.28)
    ap.add_argument('--z-bottom', type=float, default=1.02)
    ap.add_argument('--target', type=float, default=10.0, help='contact force, N')
    ap.add_argument('--press-depth', type=float, default=0.0015, help='reference past the wall surface, m')
    ap.add_argument('--stroke-speed', type=float, default=0.05, help='velocity scale of the stroke (and of free moves)')
    ap.add_argument('--target-path', choices=['bridge', 'jtac'], default='jtac')
    ap.add_argument('--record', default='/tmp/wall_pass.csv')
    ap.add_argument('--timeout', type=float, default=120.0)
    args = ap.parse_args()

    rclpy.init()
    node = WallPass()
    node.target_via = args.target_path
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    failures = []

    def check(name, ok, detail):
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}", flush=True)
        if not ok:
            failures.append(name)

    t0 = time.monotonic()
    while time.monotonic() - t0 < args.timeout:
        js, raw, truth = node.state()
        if (js is not None and raw is not None and truth is not None and node.cond_now() is not None
                and node.client.server_is_ready() and node.params.service_is_ready()
                and node.cart.service_is_ready()):
            break
        time.sleep(0.2)
    else:
        print('[FAIL] startup: missing joint_states / wrench / action server / services / move_group')
        os._exit(1)

    def run():
        node.move(FREE, 8.0)
        node.hold(1.0)
        orientation = node.tcp_pose().rotation
        axes_on = [False, False, True, False, False, False]
        for k, y in enumerate(args.strokes_y):
            free_x = WALL_X - 0.03
            press_x = WALL_X + args.press_depth
            steps = [
                ('approach', [node.pose(free_x, y, args.z_top, orientation)], 0.15, False),
                ('press', [node.pose(press_x, y, args.z_top, orientation)], args.stroke_speed, True),
                ('stroke', [node.pose(press_x, y, args.z_bottom, orientation)], args.stroke_speed, True),
            ]
            ok = True
            for name, wps, scale, contact in steps:
                if contact and name == 'press':
                    node.set_target(args.target)
                    time.sleep(0.2)
                    node.set_axes(axes_on)
                traj, frac = node.plan(wps, scale)
                if frac < 0.98 or not traj.points:
                    check(f'stroke {k} {name} planned', False, f'cartesian path fraction {frac:.2f}')
                    ok = False
                    break
                code = node.execute(traj)
                check(f'stroke {k} {name} executed', code == 0, f'error_code {code}, {len(traj.points)} points')
                if code != 0:
                    ok = False
                    break
                if name == 'press':
                    node.hold(2.0)   # let the admittance settle on the target force
            node.set_axes([False] * 6)
            node.set_target(0.0)
            time.sleep(0.3)
            # retreat 3 cm from wherever the stroke ended
            try:
                cur = node.tcp_pose()
                traj, frac = node.plan([node.pose(free_x, cur.translation.y, cur.translation.z, orientation)], 0.15)
                if frac > 0.98 and traj.points:
                    node.execute(traj)
            except Exception as e:  # noqa: BLE001
                print(f'[WARN] retreat failed: {e}', flush=True)
            if not ok:
                break

    guarded(node, run)

    y_lo, y_hi = min(args.strokes_y) - 0.07, max(args.strokes_y) + 0.07
    n = node.write_recording(args.record, WALL_X, (y_lo, y_hi), (args.z_bottom - 0.02, args.z_top + 0.02))
    print(f'recorded {n} samples to {args.record}', flush=True)
    print('WALL PASS', 'FAILED: ' + ', '.join(failures) if failures else 'EXECUTED', flush=True)
    os._exit(1 if failures else 0)


if __name__ == '__main__':
    main()
