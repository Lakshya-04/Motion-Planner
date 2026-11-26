#!/usr/bin/env python3
"""Wall docking check in the MuJoCo sim (scripts/run_wall_docking.sh starts everything).

Against a sim launched with base_motion:=true and depth_sensor:=true on the smoke wall (plane x = 0.98 m in the odom frame,
normal -x), a wall_detector with target_frame odom and the wall_docking_node:

  1. drives the base to a start pose `--offset-m` too far from the target standoff and yawed `--yaw-deg`
     (with the stand navigator's controller, from /odom), and waits for fresh scans of the wall;
  2. sends a WallDock goal (`--standoff-m`, measured from the footprint origin unless the node's ref_xy says otherwise);
  3. checks the result against the known wall and /odom: standoff error <= `--tol-m`, yaw error <= `--tol-deg`.

Prints the start pose, the final standoff and yaw error (from /odom, the simulator's own base joints, not from the
detected plane) and the time. Exit code 0 on success.
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from stand_navigator.core import BaseProfile, StandController, Tolerance, load_profile, wrap
from stand_navigator_msgs.action import WallDock
from std_msgs.msg import String

WALL_X = 0.98          # smoke wall plane in the odom frame, outward normal -x


def yaw_of(q):
    return 2.0 * math.atan2(q.z, q.w)


class Check(Node):
    def __init__(self):
        super().__init__('sim_mujoco_wall_docking_test')
        self.odom, self.scenes, self.feedback = None, 0, None
        self.truth = None   # /odom_truth, only there when the sim runs with odom_realism
        self.create_subscription(Odometry, '/odom_truth', lambda m: setattr(self, 'truth', m), 10)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, '/wall_detector/wall_scene', self._scene, latched)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.client = ActionClient(self, WallDock, 'wall_dock')

    def _scene(self, _):
        self.scenes += 1

    def spin_for(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def pose(self):
        o = self.odom.pose.pose
        return o.position.x, o.position.y, yaw_of(o.orientation)

    def speed(self):
        t = self.odom.twist.twist
        return math.hypot(t.linear.x, t.linear.y), abs(t.angular.z)

    def drive_to(self, goal, profile, timeout=60.0):
        ctl = StandController(profile, goal, Tolerance(0.01, 0.01))
        last, t0 = time.time(), time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.time()
            if now - last < 0.02:
                continue
            pose = self.pose()
            if ctl.arrived(pose, *self.speed()):
                self.pub.publish(Twist())
                return True
            vx, vy, wz = ctl.step(pose, now - last)
            last = now
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
            self.pub.publish(t)
        self.pub.publish(Twist())
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', required=True, help='robot profile YAML (the base: section)')
    ap.add_argument('--standoff-m', type=float, default=0.55)
    ap.add_argument('--offset-m', type=float, default=0.4)
    ap.add_argument('--yaw-deg', type=float, default=15.0)
    ap.add_argument('--tol-m', type=float, default=0.02)
    ap.add_argument('--tol-deg', type=float, default=1.5)
    ap.add_argument('--timeout', type=float, default=90.0)
    args = ap.parse_args()
    rclpy.init()
    node = Check()
    profile = load_profile(args.profile)
    failures = []

    def check(ok, text):
        print(('PASS  ' if ok else 'FAIL  ') + text, flush=True)
        if not ok:
            failures.append(text)

    end = time.time() + 60
    while node.odom is None and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.odom is None:
        print('FAIL  no /odom')
        return 1
    if not node.client.wait_for_server(timeout_sec=30):
        print('FAIL  wall_dock action server not available')
        return 1

    x0 = WALL_X - args.standoff_m - args.offset_m
    start = (x0, 0.0, math.radians(args.yaw_deg))
    check(node.drive_to(start, profile), f'base at the start pose ({x0:.3f}, 0, {args.yaw_deg} deg)')
    node.spin_for(1.0)
    base = node.scenes
    end = time.time() + 60
    while node.scenes < base + 2 and time.time() < end:      # two fresh fits from the stopped base
        rclpy.spin_once(node, timeout_sec=0.1)
    check(node.scenes >= base + 2, 'fresh wall scenes from the detector at the start pose')
    px, py, pyaw = node.pose()
    s0, y0 = WALL_X - px, wrap(pyaw)
    print(f'      start: standoff {s0:.3f} m (goal {args.standoff_m}), yaw error {math.degrees(y0):.2f} deg', flush=True)

    goal = WallDock.Goal()
    goal.standoff_m = args.standoff_m
    goal.timeout_s = args.timeout
    t0 = time.time()
    fut = node.client.send_goal_async(goal, feedback_callback=lambda f: setattr(node, 'feedback', f.feedback))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
    handle = fut.result()
    if handle is None or not handle.accepted:
        print('FAIL  goal rejected')
        return 1
    res_f = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res_f, timeout_sec=args.timeout + 10)
    dt = time.time() - t0
    if not res_f.done():
        print('FAIL  no result')
        return 1
    res = res_f.result().result
    node.spin_for(1.0)
    px, py, pyaw = node.pose()
    standoff, yaw_err = WALL_X - px, wrap(pyaw)
    check(res.error_code == WallDock.Result.NONE, f'result code {res.error_code} ({res.error_msg}) after {dt:.1f} s')
    print(f'      node: standoff error {res.standoff_error_m * 1000:.1f} mm, yaw error '
          f'{math.degrees(res.yaw_error_rad):.2f} deg', flush=True)
    check(abs(standoff - args.standoff_m) <= args.tol_m,
          f'final standoff {standoff:.4f} m, error {(standoff - args.standoff_m) * 1000:.1f} mm (tolerance {args.tol_m * 1000:.0f})')
    check(abs(math.degrees(yaw_err)) <= args.tol_deg,
          f'final yaw error {math.degrees(yaw_err):.2f} deg (tolerance {args.tol_deg})')
    if node.truth is not None:
        o = node.truth.pose.pose
        t_standoff, t_yaw = WALL_X - o.position.x, wrap(yaw_of(o.orientation))
        check(abs(t_standoff - args.standoff_m) <= args.tol_m,
              f'final standoff against /odom_truth {t_standoff:.4f} m, error {(t_standoff - args.standoff_m) * 1000:.1f} mm '
              f'(tolerance {args.tol_m * 1000:.0f}); reported odom differs by {(standoff - t_standoff) * 1000:.1f} mm')
        check(abs(math.degrees(t_yaw)) <= args.tol_deg,
              f'final yaw error against /odom_truth {math.degrees(t_yaw):.2f} deg (tolerance {args.tol_deg})')
    print(f'      lateral drift {py * 1000:.1f} mm; start standoff {s0:.3f} -> {standoff:.4f} m, yaw '
          f'{math.degrees(y0):.1f} -> {math.degrees(yaw_err):.2f} deg', flush=True)
    print('wall docking check:', 'PASS' if not failures else f'FAIL ({len(failures)})')
    rclpy.shutdown()
    return 0 if not failures else 1


if __name__ == '__main__':
    sys.exit(main())
