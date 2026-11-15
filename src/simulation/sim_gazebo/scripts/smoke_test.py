#!/usr/bin/env python3
"""Headless Gazebo smoke test (generic_6dof_telescopic profile, joint_trajectory_controller).

  ros2 run sim_gazebo smoke_test.py            # starts the sim itself (gz server only), then stops it

Checks, in order:
  1. gz server up: /clock advances and /joint_states arrives (robot spawned, controllers active)
  2. a joint trajectory (home -> a pose with the tool pointing at the wall, 0.3 m short of it)
     executes through FollowJointTrajectory and /joint_states ends within tolerance
  3. a second trajectory pushes the tool 3.5 cm into the wall (the world comes from
     test/data/smoke_wall_scene.json via the wall world generator). The raw F/T wrench
     (/force_torque_sensor_broadcaster/wrench, sensor frame, includes the weight of the tool) is
     converted with plain math (remove payload weight, move to the tcp with the lever arm) and
     compared with the ground-truth contact wrench (/ft_sensor/ground_truth_wrench, tcp_link)
The wall is placed with forward kinematics of the rendered URDF, so the check does not depend on
a particular simulation state. Exit code 0 only if every check passed.
"""
import argparse
import math
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
import rclpy.time
import xacro
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import WrenchStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from tf2_ros import Buffer, TransformListener

from sim_gazebo import fk
from sim_gazebo import profile as prof

PROFILE = 'generic_6dof_telescopic'
STANDOFF = 0.9
# lift + six arm joints: tool axis along +x at height 1.6 m. A: tcp x = 0.70 m (0.3 m short of the
# wall plane at x = 0.9; the tool body ends 6.5 cm before the tcp frame). B: tcp x = 1.00 m, i.e.
# 3.5 cm of the tcp frame (about 3 cm of tool body) past the plane. Verified against FK below.
POSE_A = [0.6, 0.0, 0.9797, 2.2199, -0.058, 1.5708, -0.8102]
POSE_B = [0.583, 0.0, 1.0502, 1.651, 0.4404, 1.5708, -0.8102]
TCP_A_X, TCP_B_X = 0.70, 1.00
JOINT_TOL = 0.03
FORCE_RISE_MIN_N = 5.0


class Probe(Node):
    def __init__(self, joints, controller):
        super().__init__('sim_gazebo_smoke')
        self.joints = joints
        self.sim_time = None
        self.q = {}
        self.wrench = None
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self.create_subscription(JointState, '/joint_states', self._js, 10)
        self.truth = None
        self.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster/wrench', self._w, 10)
        self.create_subscription(WrenchStamped, '/ft_sensor/ground_truth_wrench', self._gt, 10)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.client = ActionClient(self, FollowJointTrajectory,
                                   f'/{controller}/follow_joint_trajectory')

    def _on_clock(self, m):
        self.sim_time = m.clock.sec + m.clock.nanosec * 1e-9

    def _js(self, m):
        self.q = dict(zip(m.name, m.position))

    def _w(self, m):
        w = m.wrench
        self.wrench = (w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z)

    def _gt(self, m):
        w = m.wrench
        self.truth = (w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z)

    def pose(self, frame):
        t = self.tf.lookup_transform('world', frame, rclpy.time.Time()).transform
        return (_quat_matrix(t.rotation), np.array([t.translation.x, t.translation.y, t.translation.z]))

    def positions(self):
        return [self.q.get(j, math.nan) for j in self.joints]

    def spin_until(self, cond, wall_timeout):
        end = time.time() + wall_timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond():
                return True
        return False

    def sim_wait(self, seconds, wall_timeout=60.0, sample=None):
        """Spin until the sim clock advanced by `seconds`; optionally collect sample() every spin."""
        t0, out = self.sim_time, []
        end = time.time() + wall_timeout
        while time.time() < end and self.sim_time - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.02)
            if sample:
                out.append(sample())
        return out

    def wait_active(self, name, wall_timeout=60.0):
        cli = self.create_client(ListControllers, '/controller_manager/list_controllers')
        end = time.time() + wall_timeout
        while time.time() < end:
            if cli.wait_for_service(timeout_sec=1.0):
                fut = cli.call_async(ListControllers.Request())
                rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
                if fut.result() and any(c.name == name and c.state == 'active'
                                        for c in fut.result().controller):
                    return True
            time.sleep(0.5)
        return False

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


def _quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def payload(urdf):
    """Mass and centre of mass (in the sensor frame) of everything beyond the sensor.
    The tool mounts on the sensor frame with no offset (see urdf/gazebo_tags.xacro)."""
    robot = ET.fromstring(urdf)
    m_tot, mc = 0.0, np.zeros(3)
    for name in ('ft_sensor_link', 'tool_mount_link', 'tool_tip_link'):
        inertial = robot.find(f"link[@name='{name}']/inertial")
        if inertial is None:
            continue
        m = float(inertial.find('mass').get('value'))
        origin = inertial.find('origin')
        com = np.array([float(v) for v in origin.get('xyz').split()]) if origin is not None else np.zeros(3)
        m_tot += m
        mc += m * com
    return m_tot, mc / m_tot


def to_tcp(raw, node, m, com):
    """Raw sensor wrench -> wrench the environment exerts on the tool, in tcp_link axes.
    reading = f_env + m g (child_to_parent); remove the weight (and its torque about the sensor
    origin), then move the torque to the tcp with the lever arm r."""
    Rs, ps = node.pose('ft_sensor_link')
    Rt, pt = node.pose('tcp_link')
    g_s = Rs.T @ np.array([0.0, 0.0, -9.81])
    weight = m * g_s
    f = np.array(raw[:3]) - weight
    tau = np.array(raw[3:]) - np.cross(com, weight)
    r = Rs.T @ (pt - ps)
    tau_tcp = tau - np.cross(r, f)
    R_st = Rs.T @ Rt
    return np.concatenate([R_st.T @ f, R_st.T @ tau_tcp])


def tcp_x(urdf, joints, q):
    return fk.chain_pose(urdf, 'tcp_link', dict(zip(joints, q)))[0, 3]


def stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--no-launch', action='store_true', help='use a simulation that is already running')
    ap.add_argument('--log', default='/tmp/sim_gazebo_smoke.log')
    ap.add_argument('--controller', choices=['jtc', 'jtac'], default='jtc',
                    help='jtc: joint_trajectory_controller; jtac: the compliant controller (no wrench selected, so it tracks like a JTC)')
    args = ap.parse_args()
    controller = {'jtc': 'joint_trajectory_controller', 'jtac': 'combined_arm_lift_controller'}[args.controller]

    profile = prof.load_profile(PROFILE)
    joints = prof.controlled_joints(profile)
    urdf = xacro.process_file(
        os.path.join(get_package_share_directory('mobile_manipulator_description'), 'urdf',
                     'mobile_manipulator.urdf.xacro'),
        mappings=prof.xacro_mappings(profile, 'gazebo')).toxml()

    results = []

    def check(name, ok, detail=''):
        results.append(ok)
        print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)
        return ok

    check('pose A/B reach the intended tcp x (FK)',
          abs(tcp_x(urdf, joints, POSE_A) - TCP_A_X) < 0.01 and abs(tcp_x(urdf, joints, POSE_B) - TCP_B_X) < 0.01,
          f'A={tcp_x(urdf, joints, POSE_A):.3f} B={tcp_x(urdf, joints, POSE_B):.3f}')

    proc = None
    if not args.no_launch:
        scene = os.path.join(get_package_share_directory('sim_gazebo'), 'test', 'data', 'smoke_wall_scene.json')
        log = open(args.log, 'w')
        proc = subprocess.Popen(
            ['ros2', 'launch', 'sim_gazebo', 'sim.launch.py', f'wall_scene:={scene}',
             f'standoff:={STANDOFF}', f'controller:={args.controller}', 'gui:=false'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Probe(joints, controller)
    try:
        up = node.spin_until(lambda: node.sim_time is not None and not any(
            math.isnan(v) for v in node.positions()), 120)
        check('gz server up: /clock and /joint_states received', up)
        if not up:
            return 1
        t0 = node.sim_time
        node.sim_wait(1.0)
        check('sim clock advances', node.sim_time - t0 >= 1.0, f'dt={node.sim_time - t0:.2f}s')
        home = np.array(prof.home_positions(profile))
        err = np.abs(np.array(node.positions()) - home).max()
        check('robot holds the home pose under gravity', err < JOINT_TOL, f'max|q-home|={err:.4f}')
        check('F/T sensor publishes', node.spin_until(lambda: node.wrench is not None, 20))
        check(f'{controller} active, action server up',
              node.wait_active(controller) and node.client.wait_for_server(timeout_sec=30))

        ok = node.send(POSE_A, 6.0)
        node.sim_wait(1.0)
        err = np.abs(np.array(node.positions()) - np.array(POSE_A)).max()
        check('trajectory home -> A executed and /joint_states follows', ok and err < JOINT_TOL,
              f'result_ok={ok} max|q-A|={err:.4f}')

        samples = node.sim_wait(1.0, sample=lambda: node.wrench)
        free = np.array([np.linalg.norm(s[:3]) for s in samples if s])
        base = float(np.median(free)) if free.size else math.nan
        print(f'      free-space |F| median {base:.2f} N (weight of sensor + tool, raw convention)', flush=True)

        m, com = payload(urdf)
        converted, truth, raws = [], [], []

        def sample():
            if node.wrench and node.truth:
                try:
                    converted.append(to_tcp(node.wrench, node, m, com))
                except Exception:  # noqa: BLE001  (tf not ready)
                    return
                truth.append(np.array(node.truth))
                raws.append(np.array(node.wrench))
        node.send(POSE_B, 4.0)
        node.sim_wait(1.0)
        converted.clear(), truth.clear(), raws.clear()
        node.sim_wait(1.0, sample=sample)          # settled, pressing
        err = np.abs(np.array(node.positions()) - np.array(POSE_B)).max()
        check('wall blocks the tool (joints stop short of B)', err > 0.002, f'joint error to B {err:.3f} rad')
        if not truth:
            check('contact wrench samples (raw, ground truth, tf)', False)
            return 1
        gt = np.mean(truth, axis=0)
        cv = np.mean(converted, axis=0)
        raw = np.mean(raws, axis=0)
        np.set_printoptions(precision=2, suppress=True)
        print(f'      raw sensor (ft_sensor_link): {raw}', flush=True)
        print(f'      raw -> payload removed, at tcp: {cv}', flush=True)
        print(f'      ground truth (tcp_link):        {gt}', flush=True)
        check('raw wrench rises above free space', np.linalg.norm(raw[:3]) - base > FORCE_RISE_MIN_N,
              f'|F| {np.linalg.norm(raw[:3]):.1f} N vs free {base:.1f} N')
        check('pressing along tool +z gives force_z < 0 at the tcp (ground truth)', gt[2] < -FORCE_RISE_MIN_N,
              f'fz={gt[2]:.1f} N')
        ferr = np.linalg.norm(cv[:3] - gt[:3])
        terr = np.linalg.norm(cv[3:] - gt[3:])
        check('conditioned raw force matches ground truth', ferr < max(3.0, 0.15 * np.linalg.norm(gt[:3])),
              f'|dF|={ferr:.2f} N of {np.linalg.norm(gt[:3]):.1f} N')
        check('conditioned raw torque matches ground truth', terr < max(0.5, 0.2 * np.linalg.norm(gt[3:])),
              f'|dT|={terr:.2f} Nm of {np.linalg.norm(gt[3:]):.2f} Nm')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if proc:
            stop(proc)
    print('SMOKE PASS' if all(results) else 'SMOKE FAIL', flush=True)
    return 0 if all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
