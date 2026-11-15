#!/usr/bin/env python3
"""Base motion check for the MuJoCo backend. Run against a sim launched with base_motion:=true
(scripts/run_base_motion.sh does that): drives the base 0.3 m forward on /cmd_vel and checks

  1. /odom and TF odom -> base_footprint start at the origin and agree with each other;
  2. the base travels 0.3 m (+-0.03) with small lateral and yaw error, then holds still;
  3. the arm and lift joint states stay at their start pose (max deviation 0.02 rad/m);
  4. the raw F/T wrench stays sane (within 2 N of the resting reading, so no spikes from the motion);
  5. no command, no motion: after the stop the pose does not drift, and a stale command times out.

Exit code 0 on success.
"""
import argparse
import math
import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return 2.0 * math.atan2(q.z, q.w)


class BaseTest(Node):
    def __init__(self):
        super().__init__('sim_mujoco_base_motion_test')
        self.lock = threading.Lock()
        self.odom, self.js, self.raw = None, None, None
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(JointState, '/joint_states', self._js, 10)
        self.create_subscription(WrenchStamped, '/sim_mujoco/ft_raw', self._raw, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

    def cmd(self, vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
        self.pub.publish(t)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--distance', type=float, default=0.3)
    ap.add_argument('--speed', type=float, default=0.2)
    ap.add_argument('--timeout', type=float, default=90.0)
    args = ap.parse_args()
    rclpy.init()
    node = BaseTest()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    failures = []

    def check(name, ok, detail):
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}", flush=True)
        if not ok:
            failures.append(name)

    t0 = time.monotonic()
    while time.monotonic() - t0 < args.timeout:
        odom, js, raw = node.state()
        if odom is not None and js is not None and raw is not None and 'arm_elbow_joint' in js:
            break
        time.sleep(0.2)
    else:
        print('[FAIL] startup: no /odom, /joint_states or ft_raw (is the sim running with base_motion:=true?)')
        os._exit(1)
    time.sleep(2.0)   # settle

    x0, y0, yaw0 = node.pose()
    check('odom starts at the origin', math.hypot(x0, y0) < 0.01 and abs(yaw0) < 0.01,
          f'x {x0:.4f} y {y0:.4f} yaw {yaw0:.4f}')
    odom = node.state()[0]
    check('odom frames', odom.header.frame_id == 'odom' and odom.child_frame_id == 'base_footprint',
          f'{odom.header.frame_id} -> {odom.child_frame_id}')
    arm0 = {k: v for k, v in node.state()[1].items() if k.startswith(('arm_', 'lift_'))}
    f0 = node.state()[2]

    # drive forward until odom says the distance is reached
    max_dev, max_df = 0.0, 0.0
    t_start = time.monotonic()
    while time.monotonic() - t_start < 3.0 * args.distance / args.speed + 5.0:
        x, y, _ = node.pose()
        if x - x0 >= args.distance - 0.01:
            break
        node.cmd(vx=args.speed)
        js, raw = node.state()[1], node.state()[2]
        max_dev = max(max_dev, max(abs(js[k] - v) for k, v in arm0.items()))
        max_df = max(max_df, max(abs(a - b) for a, b in zip(raw, f0)))
        time.sleep(0.02)
    node.cmd()
    t_stop = time.monotonic()
    time.sleep(1.5)
    x1, y1, yaw1 = node.pose()
    travelled = x1 - x0
    check('base travelled the distance', abs(travelled - args.distance) < 0.03,
          f'{travelled:.3f} m (target {args.distance} m, took {t_stop - t_start:.1f} s)')
    check('little lateral / yaw error', abs(y1 - y0) < 0.02 and abs(yaw1 - yaw0) < 0.02,
          f'dy {y1 - y0:.4f} m, dyaw {yaw1 - yaw0:.4f} rad')

    try:
        tf = node.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        check('TF odom -> base_footprint matches /odom', math.hypot(tx - x1, ty - y1) < 0.01,
              f'tf ({tx:.3f}, {ty:.3f}) vs odom ({x1:.3f}, {y1:.3f})')
        # the arm hangs off base_footprint through robot_state_publisher: odom -> tcp is one tree
        tcp = node.tf_buffer.lookup_transform('odom', 'tcp_link', rclpy.time.Time())
        check('TF odom -> tcp_link resolves and moved with the base',
              abs(tcp.transform.translation.x - x1) < 2.0,
              f'tcp x {tcp.transform.translation.x:.3f} (base x {x1:.3f})')
    except Exception as e:  # tf2 raises several exception types
        check('TF odom -> base_footprint', False, repr(e))

    js, raw = node.state()[1], node.state()[2]
    dev = max(max_dev, max(abs(js[k] - v) for k, v in arm0.items()))
    check('arm and lift joints hold their pose', dev < 0.02, f'max deviation {dev:.4f} rad/m')
    df = max(max_df, max(abs(a - b) for a, b in zip(raw, f0)))
    check('F/T stays sane', df < 2.0 and all(abs(v) < 100.0 for v in raw),
          f'max |F/T - resting| = {df:.2f}, raw now {[round(v, 2) for v in raw]}')

    xa, ya, _ = node.pose()
    time.sleep(2.0)
    xb, yb, _ = node.pose()
    check('no command: base holds', math.hypot(xb - xa, yb - ya) < 0.005,
          f'drift {math.hypot(xb - xa, yb - ya) * 1000:.2f} mm in 2 s')
    node.cmd(vx=0.2)   # a single message must time out (0.5 s) instead of driving on
    time.sleep(1.5)
    xc, _, _ = node.pose()
    moved = xc - xb
    check('stale command times out', 0.0 < moved < 0.2 * 0.5 + 0.06,
          f'moved {moved:.3f} m from one 0.2 m/s message (<= 0.16 m)')

    print('BASE MOTION', 'FAILED: ' + ', '.join(failures) if failures else 'PASSED', flush=True)
    sys.stdout.flush()
    os._exit(1 if failures else 0)


if __name__ == '__main__':
    main()
