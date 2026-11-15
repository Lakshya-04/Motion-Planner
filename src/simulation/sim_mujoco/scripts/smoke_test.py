#!/usr/bin/env python3
"""Headless smoke test for the MuJoCo backend. Run against a launched sim (UR10e profile, the
smoke wall scene, see docs/sim_mujoco.md / scripts/run_smoke.sh):

  1. a joint trajectory executes through the controller and /joint_states follows it;
  2. in free space the ground-truth contact wrench is ~0 and the raw sensor reads only the payload;
  3. pushing the tool into the wall gives a wrench of the right sign and magnitude order, and the
     raw sensor wrench agrees with the ground truth once the payload weight and the lever arm from
     the sensor to the TCP are accounted for (plain math, below).

Exit code 0 on success. Output is one line per check.
"""
import argparse
import os
import sys
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import WrenchStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ['lift_joint', 'arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_joint',
          'arm_wrist_1_joint', 'arm_wrist_2_joint', 'arm_wrist_3_joint']
HOME = [0.4, 0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
# tool axis +x, TCP at (0.95, 0.15, 1.15), and 5 cm further along +x (wall plane at x = 0.98)
FREE = [0.4, -0.0363, 0.2528, 1.1964, 1.6924, -1.5345, 1.5708]
PRESS = [0.4, -0.0324, 0.2817, 1.0151, 1.8448, -1.5384, 1.5708]


class Smoke(Node):
    def __init__(self, controller):
        super().__init__('sim_mujoco_smoke')
        self.lock = threading.Lock()
        self.js, self.raw, self.truth = None, None, None
        self.log = []   # (t, q)
        self.create_subscription(JointState, '/joint_states', self._js, 10)
        self.create_subscription(WrenchStamped, '/sim_mujoco/ft_raw', self._raw, qos_profile_sensor_data)
        self.create_subscription(WrenchStamped, '/sim_mujoco/ft_ground_truth', self._truth,
                                 qos_profile_sensor_data)
        self.client = ActionClient(self, FollowJointTrajectory,
                                   f'/{controller}/follow_joint_trajectory')

    def _js(self, m):
        q = dict(zip(m.name, m.position))
        if all(j in q for j in JOINTS):
            with self.lock:
                self.js = [q[j] for j in JOINTS]
                self.log.append((time.monotonic(), self.js))

    @staticmethod
    def _w(m):
        w = m.wrench
        return [w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z]

    def _raw(self, m):
        with self.lock:
            self.raw = self._w(m)

    def _truth(self, m):
        with self.lock:
            self.truth = self._w(m)

    def state(self):
        with self.lock:
            return self.js, self.raw, self.truth

    def move(self, target, seconds):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        start = self.state()[0]
        for frac in (0.5, 1.0):  # midpoint with rest-to-rest timing keeps the spline well behaved
            p = JointTrajectoryPoint()
            p.positions = [a + (b - a) * frac for a, b in zip(start, target)]
            p.velocities = [0.0] * 7 if frac == 1.0 else [(b - a) / seconds * 1.5 for a, b in zip(start, target)]
            p.time_from_start.sec = int(seconds * frac)
            p.time_from_start.nanosec = int((seconds * frac % 1) * 1e9)
            goal.trajectory.points.append(p)
        # the action server comes up when the controller is configured, a moment before it is active
        for _ in range(100):
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

    def hold(self, seconds):
        t0 = time.monotonic()
        samples = []
        while time.monotonic() - t0 < seconds:
            time.sleep(0.02)
            samples.append(self.state())
        return samples


def mean(rows):
    return [sum(r[i] for r in rows) / len(rows) for i in range(len(rows[0]))]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--controller', default='combined_arm_lift_controller')
    ap.add_argument('--tcp-z', type=float, default=0.165, help='sensor-to-TCP offset along z')
    ap.add_argument('--timeout', type=float, default=90.0)
    ap.add_argument('--release-timeout', type=float, default=8.0,
                    help='how long after the retreat trajectory ends the force may take to vanish')
    args = ap.parse_args()
    rclpy.init()
    node = Smoke(args.controller)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    failures = []

    def check(name, ok, detail):
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}", flush=True)
        if not ok:
            failures.append(name)

    t0 = time.monotonic()
    while time.monotonic() - t0 < args.timeout:
        js, raw, truth = node.state()
        if js is not None and raw is not None and truth is not None and node.client.server_is_ready():
            break
        time.sleep(0.2)
    else:
        print('[FAIL] startup: no joint_states / wrench topics / action server')
        os._exit(1)

    # 1. trajectory execution, joint_states follow
    start = node.state()[0]
    check('start pose', max(abs(a - b) for a, b in zip(start, HOME)) < 0.02,
          f'max |q - home| = {max(abs(a - b) for a, b in zip(start, HOME)):.4f}')
    t_move = time.monotonic()
    code = node.move(FREE, 8.0)
    t_end = time.monotonic()
    time.sleep(1.0)
    q = node.state()[0]
    err = max(abs(a - b) for a, b in zip(q, FREE))
    check('trajectory result', code == 0, f'error_code {code}')
    check('joint_states reach the goal', err < 0.02, f'max |q - goal| = {err:.4f} rad/m')
    with node.lock:
        mid = [row for t, row in node.log if abs(t - (t_move + 4.0)) < 0.06]
    if mid:
        between = all(min(a, b) - 0.05 <= m <= max(a, b) + 0.05 for m, a, b in zip(mid[0], start, FREE))
        check('joint_states follow in flight', between, 'mid-trajectory state lies between start and goal')
    else:
        check('joint_states follow in flight', False, 'no joint_states sample at mid-trajectory')
    check('trajectory took the commanded time', 6.0 < t_end - t_move < 12.0, f'{t_end - t_move:.1f} s')

    # 2. free space: no contact, raw = payload only
    free = node.hold(2.0)
    truth_free = mean([s[2] for s in free])
    w0 = mean([s[1] for s in free])
    check('free space ground truth ~ 0', max(abs(v) for v in truth_free[:3]) < 0.5,
          f'truth F = {[round(v, 3) for v in truth_free[:3]]} N')
    check('raw sensor reads the payload weight', 3.0 < sum(v * v for v in w0[:3]) ** 0.5 < 8.0,
          f'|F_raw| = {sum(v * v for v in w0[:3]) ** 0.5:.2f} N (0.5 kg tool: 4.9 N), '
          f'F = {[round(v, 2) for v in w0[:3]]}')

    # 3. press into the wall
    code = node.move(PRESS, 4.0)
    node.hold(1.5)
    rows = node.hold(1.5)
    raw = mean([s[1] for s in rows])
    truth = mean([s[2] for s in rows])
    lateral = (truth[0] ** 2 + truth[1] ** 2) ** 0.5
    check('press trajectory finished', code == 0, f'error_code {code}')
    check('contact wrench sign (tool pushes +z, wall pushes back)', truth[2] < -2.0,
          f'truth Fz = {truth[2]:.1f} N')
    check('contact wrench magnitude order', 2.0 < abs(truth[2]) < 1000.0, f'|Fz| = {abs(truth[2]):.1f} N')
    check('force is along the tool axis', abs(truth[2]) > 2.0 * lateral,
          f'Fz {truth[2]:.1f} N, lateral {lateral:.1f} N')
    # raw = truth + payload at the sensor; torque about the sensor = torque at TCP + r x F + payload torque
    r = [0.0, 0.0, args.tcp_z]
    lever = cross(r, truth[:3])
    exp_f = [t + w for t, w in zip(truth[:3], w0[:3])]
    exp_t = [truth[3 + i] + lever[i] + w0[3 + i] for i in range(3)]
    tol_f = 0.08 * abs(truth[2]) + 0.5
    tol_t = 0.08 * abs(truth[2]) * args.tcp_z + 0.3
    df = max(abs(a - b) for a, b in zip(raw[:3], exp_f))
    dt = max(abs(a - b) for a, b in zip(raw[3:], exp_t))
    check('raw force = truth + payload', df < tol_f,
          f'raw {[round(v, 2) for v in raw[:3]]} vs {[round(v, 2) for v in exp_f]} (max diff {df:.2f} N)')
    check('raw torque = truth torque + lever arm x force + payload torque', dt < tol_t,
          f'raw {[round(v, 2) for v in raw[3:]]} vs {[round(v, 2) for v in exp_t]} (max diff {dt:.2f} N m)')

    # retreat: the force goes away. The controller decides how fast: a JointTrajectoryController follows
    # the reference at once, JTAC's admittance rule relaxes its compliance offset (accumulated while the
    # wall blocked the tool) with time constant ~D/K = 2 s even with no axis selected, so the commanded
    # pose stays inside the wall for ~3 s after the reference left (docs/sim_mujoco.md, "Retreat").
    node.move(FREE, 4.0)
    t_ret, released, quiet_since = time.monotonic(), None, None
    while time.monotonic() - t_ret < args.release_timeout:
        now = time.monotonic()
        if abs(node.state()[2][2]) < 0.5:   # released = quiet for 0.5 s (a single zero sample is a bounce)
            quiet_since = quiet_since or now
            if now - quiet_since >= 0.5:
                released = quiet_since - t_ret
                break
        else:
            quiet_since = None
        time.sleep(0.02)
    rel = mean([s[2] for s in node.hold(1.0)])
    check('force released on retreat', released is not None and abs(rel[2]) < 0.5,
          f'truth |Fz| < 0.5 N {released:.2f} s after the retreat trajectory ended, then Fz = {rel[2]:.2f} N'
          if released is not None else f'still {node.state()[2][2]:.1f} N after {args.release_timeout:.0f} s')

    print('SMOKE', 'FAILED: ' + ', '.join(failures) if failures else 'PASSED', flush=True)
    sys.stdout.flush()
    os._exit(1 if failures else 0)  # rclpy's spin thread would otherwise abort at interpreter exit


if __name__ == '__main__':
    main()
