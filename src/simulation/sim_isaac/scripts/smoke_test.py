#!/usr/bin/env python3
"""ROS-side smoke test for the Isaac add-on. Run while run_isaac.py is up (see scripts/run_smoke.sh):

  1. /clock and /joint_states arrive;
  2. joint names and order are the profile's (plus the lift's mimic stages, nothing else);
  3. the arm holds its start pose under gravity with no command, mimic stages follow the lift;
  4. a JointState command for one joint reaches its target and holds.

Exit code 0 on success, one PASS/FAIL line per check.
"""
import argparse
import os
import pathlib
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import isaac_common as ic  # noqa: E402


def profile_dirs():
    dirs = [os.environ['MP_PROFILE_DIR']] if os.environ.get('MP_PROFILE_DIR') else []
    for p in HERE.parents:
        cand = p / 'src' / 'description' / 'mobile_manipulator_description' / 'config' / 'profiles'
        if cand.is_dir():
            dirs.append(cand)
    try:
        from ament_index_python.packages import get_package_share_directory
        dirs.append(pathlib.Path(get_package_share_directory('mobile_manipulator_description')) / 'config' / 'profiles')
    except Exception:
        pass
    return dirs


class Probe(Node):
    def __init__(self, command_topic):
        super().__init__('sim_isaac_smoke')
        self.lock = threading.Lock()
        self.sim_clock = None
        self.js = None
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)
        self.pub = self.create_publisher(JointState, command_topic, 10)

    def _on_clock(self, m):
        with self.lock:
            self.sim_clock = m.clock.sec + m.clock.nanosec * 1e-9

    def _on_js(self, m):
        with self.lock:
            self.js = (list(m.name), list(m.position))

    def sim_time(self):
        with self.lock:
            return self.sim_clock

    def state(self):
        with self.lock:
            return self.js

    def positions(self):
        s = self.state()
        return dict(zip(*s)) if s else {}

    def wait_sim(self, seconds, timeout=60.0):
        """Wait `seconds` of simulated time (wall timeout guards a stalled sim)."""
        t0 = self.sim_time()
        wall = time.monotonic()
        while self.sim_time() - t0 < seconds:
            if time.monotonic() - wall > timeout:
                return False
            time.sleep(0.01)
        return True

    def command(self, joint, value):
        m = JointState()
        m.name = [joint]
        m.position = [float(value)]
        self.pub.publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', default=ic.DEFAULT_PROFILE)
    ap.add_argument('--command-topic', default=ic.DEFAULT_COMMAND_TOPIC)
    ap.add_argument('--joint', default='arm_wrist_3_joint', help='joint to command')
    ap.add_argument('--delta', type=float, default=0.5, help='command offset from the start pose (rad or m)')
    ap.add_argument('--tol', type=float, default=0.02)
    ap.add_argument('--hold-tol', type=float, default=0.01)
    ap.add_argument('--mimic-ratio', type=float, default=0.5, help='mimic stage / lift position')
    ap.add_argument('--timeout', type=float, default=120.0, help='wall seconds to wait for the sim topics')
    args = ap.parse_args()

    profile = ic.load_profile(ic.find_profile(args.profile, profile_dirs()))
    joints = ic.joint_order(profile)
    mimics = ic.mimic_joint_names(profile)
    home = dict(zip(joints, ic.home_positions(profile, joints)))
    if args.joint not in joints:
        sys.exit(f'--joint {args.joint} is not in the profile joints {joints}')

    rclpy.init()
    node = Probe(args.command_topic)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    failures = []

    def check(name, ok, detail):
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}", flush=True)
        if not ok:
            failures.append(name)

    t0 = time.monotonic()
    while time.monotonic() - t0 < args.timeout and (node.sim_time() is None or node.state() is None):
        time.sleep(0.2)
    if node.sim_time() is None or node.state() is None:
        print('[FAIL] startup: no /clock or /joint_states within the timeout', flush=True)
        os._exit(1)
    check('topics', True, f'/clock at {node.sim_time():.2f} s, /joint_states with {len(node.state()[0])} joints')

    names = node.state()[0]
    ours = [n for n in names if n in joints]
    extra = [n for n in names if n not in joints]
    check('joint names and order', ours == joints and sorted(extra) == sorted(mimics),
          f'profile joints {ours == joints} in order, extra {extra} (profile mimic {mimics})')

    node.wait_sim(0.5)
    samples = []
    for _ in range(30):
        node.wait_sim(0.1)
        samples.append(node.positions())
    worst = max(abs(s[j] - home[j]) for s in samples for j in joints)
    drift = max(max(s[j] for s in samples) - min(s[j] for s in samples) for j in joints)
    check('arm holds start pose', worst < args.hold_tol, f'max |q - home| = {worst:.4f}, max drift {drift:.5f} over 3 s')

    if mimics and 'lift_joint' in home:
        lift = samples[-1][profile['joints']['lift']]
        err = max(abs(samples[-1][m] - args.mimic_ratio * lift) for m in mimics)
        check('mimic stages follow the lift', err < args.hold_tol, f'max |stage - {args.mimic_ratio} * lift| = {err:.4f}')

    target = home[args.joint] + args.delta
    t_cmd = node.sim_time()
    reached = None
    while node.sim_time() - t_cmd < 15.0:
        node.command(args.joint, target)
        time.sleep(0.05)
        if abs(node.positions()[args.joint] - target) < args.tol:
            reached = node.sim_time() - t_cmd
            break
    q = node.positions()[args.joint]
    check('command reaches target', reached is not None,
          f'{args.joint} = {q:.4f} for target {target:.4f}' + (f' after {reached:.2f} s sim time' if reached else ''))

    node.wait_sim(2.0)
    held = [node.positions() for _ in range(20) if node.wait_sim(0.1) or True]
    err = max(abs(s[args.joint] - target) for s in held)
    others = max(abs(s[j] - home[j]) for s in held for j in joints if j != args.joint)
    check('target holds after the command stops', err < args.tol, f'max |q - target| = {err:.4f} over 2 s')
    check('other joints stay put', others < args.hold_tol, f'max |q - home| = {others:.4f}')

    print('SMOKE', 'PASS' if not failures else 'FAIL: ' + ', '.join(failures), flush=True)
    os._exit(1 if failures else 0)


if __name__ == '__main__':
    main()
