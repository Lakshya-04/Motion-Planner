#!/usr/bin/env python3
"""Compare the effort Gazebo reports for each joint at rest with gravity_core's torque for the same pose.

  ros2 run sim_gazebo gravity_check.py [--profile ur10e_telescopic] [--poses 4]

Starts the sim (gz server only), drives home and a few poses through the joint trajectory controller,
waits for the arm to settle, then prints measured effort, gravity_core torque and the difference per
joint. gravity_core runs as its own CLI (lib/gravity_core/gravity_eval) on the rendered URDF.
Gazebo holds position-commanded joints with a velocity motor, so the pose is held whatever the load;
this checks the model against what the physics engine says each joint carries, it does not tune a loop.
Exit code 0 when every joint is within --tol (N m, N for the lift).
"""
import argparse
import math
import os
import signal
import subprocess
import sys
import time

import xml.etree.ElementTree as ET

import numpy as np
import rclpy
import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from sim_gazebo import profile as prof

CONTROLLER = 'joint_trajectory_controller'


class Probe(Node):
    def __init__(self, joints):
        super().__init__('sim_gazebo_gravity_check')
        self.joints = joints
        self.sim_time = None
        self.q = {}
        self.effort = {}
        self.create_subscription(Clock, '/clock', lambda m: setattr(
            self, 'sim_time', m.clock.sec + m.clock.nanosec * 1e-9), 10)
        self.create_subscription(JointState, '/joint_states', self._js, 10)
        self.client = ActionClient(self, FollowJointTrajectory, f'/{CONTROLLER}/follow_joint_trajectory')

    def _js(self, m):
        self.q = dict(zip(m.name, m.position))
        if len(m.effort) == len(m.name):
            self.effort = dict(zip(m.name, m.effort))

    def spin_until(self, cond, wall_timeout):
        end = time.time() + wall_timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond():
                return True
        return False

    def sim_wait(self, seconds, sample=None):
        t0, out, end = self.sim_time, [], time.time() + 120
        while time.time() < end and self.sim_time - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.02)
            if sample:
                out.append(sample())
        return out

    def send(self, target, seconds):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joints
        pt = JointTrajectoryPoint(positions=[float(v) for v in target], velocities=[0.0] * len(target))
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return False
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=seconds * 6 + 20)
        return res.result() is not None and res.result().result.error_code == 0


def mimic_joints(urdf_text):
    """(name, driver, multiplier, offset) for every <mimic> joint."""
    out = []
    for j in ET.fromstring(urdf_text).findall('joint'):
        m = j.find('mimic')
        if m is not None:
            out.append((j.get('name'), m.get('joint'), float(m.get('multiplier', 1)), float(m.get('offset', 0))))
    return out


def gravity_torques(eval_bin, urdf_path, joints, poses, mimics):
    """gravity_core torque per joint, with each mimic stage as its own output.

    Gazebo reports the effort of the lift joint alone; gravity_core by default folds the stages' load into
    the joint they follow (what a single actuator has to supply). Listing the stages as outputs keeps them apart.
    """
    names = list(joints) + [m[0] for m in mimics]
    rows = []
    for q in poses:
        extra = [mult * q[joints.index(drv)] + off for _, drv, mult, off in mimics]
        rows.append(list(q) + extra)
    text = '\n'.join(' '.join(repr(float(v)) for v in r) for r in rows) + '\n'
    out = subprocess.run([eval_bin, urdf_path, ','.join(names)], input=text, capture_output=True,
                         text=True, check=True).stdout
    return np.array([[float(v) for v in line.split()] for line in out.strip().splitlines()])[:, :len(joints)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', default='ur10e_telescopic')
    ap.add_argument('--poses', type=int, default=4, help='random poses besides home')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--tol', type=float, default=2.0, help='N m (N for the lift)')
    ap.add_argument('--no-launch', action='store_true')
    ap.add_argument('--log', default='/tmp/sim_gazebo_gravity_check.log')
    args = ap.parse_args()

    profile = prof.load_profile(args.profile)
    joints = prof.controlled_joints(profile)
    urdf = xacro.process_file(
        os.path.join(get_package_share_directory('mobile_manipulator_description'), 'urdf',
                     'mobile_manipulator.urdf.xacro'),
        mappings=prof.xacro_mappings(profile, 'gazebo')).toxml()
    urdf_path = '/tmp/gravity_check_robot.urdf'
    open(urdf_path, 'w').write(urdf)
    eval_bin = os.path.join(get_package_prefix('gravity_core'), 'lib', 'gravity_core', 'gravity_eval')

    home = np.array(prof.home_positions(profile))
    rng = np.random.default_rng(args.seed)
    # modest moves around home so the arm stays clear of the floor and its own base
    poses = [home] + [home + rng.uniform(-0.5, 0.5, size=home.size) * np.r_[0.3, np.ones(home.size - 1)]
                      for _ in range(args.poses)]

    proc = None
    if not args.no_launch:
        proc = subprocess.Popen(['ros2', 'launch', 'sim_gazebo', 'sim.launch.py', f'profile:={args.profile}',
                                 'gui:=false'],
                                stdout=open(args.log, 'w'), stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Probe(joints)
    ok_all = True
    try:
        if not node.spin_until(lambda: node.sim_time is not None and len(node.q) >= len(joints), 120):
            print('FAIL  no /joint_states')
            return 1
        if not node.client.wait_for_server(timeout_sec=60):
            print('FAIL  no trajectory action server')
            return 1
        measured = []
        reached = []
        for q in poses:
            ok = node.send(q, 5.0)
            node.sim_wait(2.0)
            samples = node.sim_wait(1.0, sample=lambda: [node.effort.get(j, math.nan) for j in joints])
            measured.append(np.nanmean(np.array(samples), axis=0))
            reached.append([node.q.get(j, math.nan) for j in joints])
            print(f'pose reached={ok} max|q-target|={np.abs(np.array(reached[-1]) - q).max():.4f}', flush=True)
        reached = np.array(reached)
        measured = np.array(measured)
        # evaluate the model at the poses the arm actually settled at
        model = gravity_torques(eval_bin, urdf_path, joints, reached, mimic_joints(urdf))
        diff = measured - model
        print(f'{"joint":32s} {"max |measured|":>14} {"max |model|":>12} {"max |diff|":>11}')
        for k, j in enumerate(joints):
            print(f'{j:32s} {np.abs(measured[:, k]).max():14.3f} {np.abs(model[:, k]).max():12.3f} '
                  f'{np.abs(diff[:, k]).max():11.3f}')
        if np.nanmax(np.abs(measured)) < 1e-6:
            print('NOTE  Gazebo reports zero effort for every joint: no reference to compare against')
        ok_all = bool(np.nanmax(np.abs(diff)) < args.tol)
        print('gravity check:', 'PASS' if ok_all else 'FAIL', f'(worst |diff| {np.nanmax(np.abs(diff)):.3f}, tol {args.tol})')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if proc is not None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(30)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
    return 0 if ok_all else 1


if __name__ == '__main__':
    sys.exit(main())
