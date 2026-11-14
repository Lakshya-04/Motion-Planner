#!/usr/bin/env python3
"""Contact-controlled test in Gazebo: JTAC holds a target force against the wall.

  ros2 run sim_gazebo contact_control_test.py                  # starts the sim itself (headless)
  ros2 run sim_gazebo contact_control_test.py --no-launch      # against `sim.launch.py controller:=jtac
                                                               #   ft_conditioner:=true wall_scene:=...`

The whole force path runs: raw gz wrench (ft_sensor_link, includes the weight of the tool) ->
ft_conditioner (calibrated here with ~/capture_pose and ~/identify) -> ft_bridge_hardware sensor
`ft_sensor_tcp` (biased by the target on its ~/target_wrench) -> JTAC with the tool-z admittance
axis engaged (parameters set live, as JtacControllerCompliancePort does).

Steps:
  1. calibration: six still, contact-free wrist poses, capture + identify; mass and centre of
     gravity are compared with the inertials in the URDF (the known tool)
  2. free space at pose A: the conditioned wrench must be about zero (payload compensation)
  3. engage tool-z admittance with the target force, approach, press against the wall; steady
     contact force is checked against the ground truth (/ft_sensor/ground_truth_wrench, tcp_link,
     tool collision only), and the conditioned wrench against the ground truth (force, torque)
  4. target to zero, disengage, retreat: the contact force must be released
Exit code 0 only if every check passed.
"""
import argparse
import math
import os
import re
import subprocess
import sys

import numpy as np
import rclpy
import rclpy.time
import xacro
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Wrench, WrenchStamped
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

import smoke_test as st
from sim_gazebo import fk
from sim_gazebo import profile as prof

JTAC = 'combined_arm_lift_controller'
SETPOINT_TOPIC = '/ft_bridge_hardware_ft_sensor_tcp/target_wrench'
# wrist poses around pose A (the tool 0.3 m short of the wall): tool pitched up and down and rolled
# about its axis, so gravity points in clearly different directions in the sensor frame
CAL_DELTAS = [(), ((4, -0.8),), ((4, 0.8),), ((6, 0.8),), ((6, -0.8),), ((4, 0.8), (6, 0.8))]


def vec(w):
    return np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])


class ContactProbe(st.Probe):
    def __init__(self, joints, controller):
        super().__init__(joints, controller)
        self.cond_log, self.truth_log = [], []      # (sim stamp, 6-vector)
        self.create_subscription(WrenchStamped, '/ft_conditioner/wrench_conditioned', self._cond,
                                 qos_profile_sensor_data)
        self.create_subscription(WrenchStamped, '/ft_sensor/ground_truth_wrench', self._truth, 50)
        self.set_params = self.create_client(SetParameters, f'/{JTAC}/set_parameters')
        self.setpoint = self.create_publisher(
            Wrench, SETPOINT_TOPIC, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                               reliability=ReliabilityPolicy.RELIABLE))
        self.capture = self.create_client(Trigger, '/ft_conditioner/capture_pose')
        self.identify = self.create_client(Trigger, '/ft_conditioner/identify')

    @staticmethod
    def _stamp(m):
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def _cond(self, m):
        self.cond_log.append((self._stamp(m), vec(m.wrench)))

    def _truth(self, m):
        self.truth_log.append((self._stamp(m), vec(m.wrench)))

    def call(self, client, req=None, timeout=15.0):
        req = req or Trigger.Request()
        client.wait_for_service(timeout_sec=timeout)
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def set_jtac(self, **params):
        req = SetParameters.Request()
        req.parameters = [Parameter(k.replace('__', '.'), value=v).to_parameter_msg()
                          for k, v in params.items()]
        self.set_params.wait_for_service(timeout_sec=10.0)
        fut = self.set_params.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        return fut.result() is not None and all(r.successful for r in fut.result().results)

    def set_max_offset(self, metres):
        pub = self.create_publisher(Float64, f'/{JTAC}/max_offset_m', 10)
        for _ in range(5):
            pub.publish(Float64(data=float(metres)))
            self.sim_wait(0.1)

    def target(self, fz):
        """Contact force `fz` along the tool z at the tcp (negative: pressing). The bridge exports
        measured - setpoint, so the admittance drives the measurement to this value."""
        w = Wrench()
        w.force.z = float(fz)
        self.setpoint.publish(w)


def window(log, t0, t1):
    t = np.array([s for s, _ in log])
    v = np.array([x for _, x in log])
    sel = (t >= t0) & (t <= t1)
    return t[sel], v[sel]


def align(cond_log, truth_log, t0, t1):
    """Conditioned samples in [t0, t1] with the ground truth interpolated to their stamps."""
    tc, vc = window(cond_log, t0, t1)
    tt = np.array([s for s, _ in truth_log])
    vt = np.array([x for _, x in truth_log])
    gt = np.stack([np.interp(tc, tt, vt[:, k]) for k in range(6)], axis=1)
    return tc, vc, gt


def stats(err):
    n = np.linalg.norm(err, axis=1)
    return dict(mean=err.mean(axis=0), std=err.std(axis=0), norm_mean=n.mean(), norm_max=n.max(),
                rms=math.sqrt((n ** 2).mean()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--no-launch', action='store_true')
    ap.add_argument('--log', default='/tmp/sim_gazebo_contact.log')
    ap.add_argument('--csv', default='', help='write the contact time series (t, conditioned, truth) here')
    ap.add_argument('--target-n', type=float, default=10.0, help='contact force to hold (N, pressing)')
    ap.add_argument('--stiffness', type=float, default=300.0, help='admittance stiffness along tool z (N/m)')
    ap.add_argument('--mass', type=float, default=8.0, help='admittance mass along tool z (kg)')
    ap.add_argument('--damping-ratio', type=float, default=6.0)
    ap.add_argument('--max-offset-m', type=float, default=0.003, help='JTAC Cartesian offset clamp (m)')
    ap.add_argument('--require-hold', action='store_true',
                    help='count the force-hold result as a check (it is only reported by default)')
    ap.add_argument('--wall-tcp-x', type=float, default=0.965, help='tcp x where the tool touches the wall (m)')
    ap.add_argument('--ref-penetration', type=float, default=0.0,
                    help='reference pose this far (m) beyond the wall surface (the reference sets the force: '
                         'F ~ k_wall (F_t + K e)/(K + k_wall))')
    ap.add_argument('--approach-speed', type=float, default=0.002, help='last centimetre of the approach (m/s)')
    ap.add_argument('--hold-s', type=float, default=8.0)
    ap.add_argument('--force-tol-n', type=float, default=2.0, help='|steady force - target| tolerance (N)')
    ap.add_argument('--force-std-n', type=float, default=1.0, help='steady force standard deviation limit (N)')
    ap.add_argument('--cond-force-tol-n', type=float, default=1.0, help='conditioned-vs-truth mean |dF| (N)')
    ap.add_argument('--cond-torque-tol-nm', type=float, default=0.2, help='conditioned-vs-truth mean |dT| (Nm)')
    args = ap.parse_args()

    profile = prof.load_profile(st.PROFILE)
    joints = prof.controlled_joints(profile)
    urdf = xacro.process_file(
        os.path.join(get_package_share_directory('mobile_manipulator_description'), 'urdf',
                     'mobile_manipulator.urdf.xacro'),
        mappings=prof.xacro_mappings(profile, 'gazebo')).toxml()
    A, B = np.array(st.POSE_A), np.array(st.POSE_B)

    def tcp_x_at(s):
        return fk.chain_pose(urdf, 'tcp_link', dict(zip(joints, A + (B - A) * s)))[0, 3]

    def pose_at_x(x):
        """Joint pose on the A-B joint-space line whose tcp x is `x` (bisection on FK)."""
        lo, hi = 0.0, 1.0
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if tcp_x_at(mid) < x else (lo, mid)
        return A + (B - A) * 0.5 * (lo + hi)

    results = []

    def check(name, ok, detail=''):
        results.append(bool(ok))
        print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)
        return ok

    proc = None
    if not args.no_launch:
        scene = os.path.join(get_package_share_directory('sim_gazebo'), 'test', 'data', 'smoke_wall_scene.json')
        proc = subprocess.Popen(
            ['ros2', 'launch', 'sim_gazebo', 'sim.launch.py', f'wall_scene:={scene}',
             f'standoff:={st.STANDOFF}', 'controller:=jtac', 'ft_conditioner:=true', 'gui:=false'],
            stdout=open(args.log, 'w'), stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = ContactProbe(joints, JTAC)
    np.set_printoptions(precision=3, suppress=True)
    try:
        up = node.spin_until(lambda: node.sim_time is not None and not any(
            math.isnan(v) for v in node.positions()), 120)
        check('gz server up', up)
        if not up:
            return 1
        check('JTAC active, action server up',
              node.wait_active(JTAC) and node.client.wait_for_server(timeout_sec=30))
        node.target(0.0)
        node.spin_until(lambda: bool(node.cond_log) and bool(node.truth_log), 20)

        # 1. calibration of the conditioner against the known tool
        for i, delta in enumerate(CAL_DELTAS):
            q = A.copy()
            for j, d in delta:
                q[j] += d
            ok = node.send(q, 5.0)
            node.sim_wait(1.5)
            r = node.call(node.capture)
            if not (ok and r and r.success):
                check(f'calibration pose {i + 1} captured', False, str(r))
        res = node.call(node.identify)
        print(f'      identify: {res.message if res else None}', flush=True)
        m_true, com_true = st.payload(urdf)
        fit = re.search(r'payload_mass_kg: ([-\d.e+]+)\npayload_cog_xyz: \[([^\]]+)\]\nbias: \[([^\]]+)\]',
                        res.message if res else '')
        if check('~/identify succeeded', bool(res and res.success and fit)):
            m_fit = float(fit.group(1))
            com_fit = np.array([float(v) for v in fit.group(2).split(',')])
            bias = np.array([float(v) for v in fit.group(3).split(',')])
            check('identified mass matches the URDF inertials', abs(m_fit - m_true) < 0.02,
                  f'fit {m_fit:.4f} kg, known {m_true:.4f} kg')
            check('identified centre of gravity matches', np.linalg.norm(com_fit - com_true) < 0.01,
                  f'fit {com_fit}, known {np.round(com_true, 4)}')
            check('identified bias is about zero (the sim sensor has none)',
                  np.abs(bias[:3]).max() < 0.1 and np.abs(bias[3:]).max() < 0.02, f'{bias}')

        # 2. free space at A with the calibrated conditioner
        node.send(A, 5.0)
        node.sim_wait(1.5)
        t0 = node.sim_time
        node.sim_wait(2.0)
        _, cf, gf = align(node.cond_log, node.truth_log, t0, node.sim_time)
        free = np.abs(cf).max(axis=0)
        check('free space: conditioned wrench is zero (payload removed)',
              free[:3].max() < 0.15 and free[3:].max() < 0.03,
              f'max|F|={free[:3].max():.3f} N max|T|={free[3:].max():.4f} Nm')

        wall_x = args.wall_tcp_x
        K = args.stiffness

        def contact_stats(label, a, b, tol=True):
            """Conditioned wrench vs ground truth in [a, b] (steady contact)."""
            _, vc, gt = align(node.cond_log, node.truth_log, a, b)
            sf, sT = stats(vc[:, :3] - gt[:, :3]), stats(vc[:, 3:] - gt[:, 3:])
            print(f'      {label}: ground truth F {gt[:, :3].mean(axis=0)} N, T {gt[:, 3:].mean(axis=0)} Nm; '
                  f'conditioned - truth over {len(vc)} samples: force mean {sf["mean"]} std {sf["std"]} '
                  f'|dF| mean {sf["norm_mean"]:.3f} max {sf["norm_max"]:.3f} N; torque mean {sT["mean"]} '
                  f'std {sT["std"]} |dT| mean {sT["norm_mean"]:.4f} max {sT["norm_max"]:.4f} Nm', flush=True)
            if tol:
                check(f'{label}: conditioned force matches ground truth (mean |dF| <= {args.cond_force_tol_n} N)',
                      sf['norm_mean'] <= args.cond_force_tol_n, f'{sf["norm_mean"]:.3f} N')
                check(f'{label}: conditioned torque matches ground truth (mean |dT| <= {args.cond_torque_tol_nm} Nm)',
                      sT['norm_mean'] <= args.cond_torque_tol_nm, f'{sT["norm_mean"]:.4f} Nm')
            return sf, sT

        # 3. conditioner accuracy in steady contact: JTAC without a wrench axis (position hold)
        #    presses 5 mm and 35 mm past the wall; the contact is steady and large, the case in which
        #    the lever arm and the payload term are both exercised
        node.send(pose_at_x(wall_x - 0.02), 8.0)
        for depth in (0.005, 0.035):
            node.send(pose_at_x(wall_x + depth), 4.0)
            node.sim_wait(2.0)
            t1 = node.sim_time
            node.sim_wait(2.0)
            _, g = window(node.truth_log, t1, node.sim_time)
            print(f'      position hold {depth * 1000:.0f} mm past the wall: ground-truth fz {g[:, 2].mean():.2f} N '
                  f'(std {g[:, 2].std():.3f})', flush=True)
            contact_stats(f'steady contact, {depth * 1000:.0f} mm past the wall', t1, node.sim_time)
        node.send(pose_at_x(wall_x - 0.02), 5.0)

        # 4. engage the tool-z admittance with the force target and press
        node.set_max_offset(args.max_offset_m)
        node.target(-args.target_n)
        ok = node.set_jtac(admittance__mass=[16.0, 16.0, args.mass, 0.5, 0.5, 0.5],
                           admittance__stiffness=[1000.0, 1000.0, K, 50.0, 50.0, 50.0],
                           admittance__damping_ratio=[5.59, 5.59, args.damping_ratio, 1.0, 1.0, 1.0],
                           admittance__selected_axes=[False, False, True, False, False, False])
        check('tool-z admittance engaged (parameters set live)', ok)
        node.sim_wait(1.0)
        node.send(pose_at_x(wall_x - 0.01), 4.0)
        node.sim_wait(1.0)
        contact_from = node.sim_time
        final = pose_at_x(wall_x + args.ref_penetration)
        node.send(final, (wall_x + args.ref_penetration - (wall_x - 0.01)) / args.approach_speed)
        node.sim_wait(args.hold_s)
        t_end = node.sim_time
        t_steady = t_end - 4.0
        _, gt_c = window(node.truth_log, contact_from, t_end)
        _, gt_s = window(node.truth_log, t_steady, t_end)
        fz = -gt_s[:, 2]
        duty = float((fz > 1.0).mean())
        print(f'      force control, target {args.target_n:.1f} N: ground-truth force over the last 4 s: mean {fz.mean():.2f} N, '
              f'std {fz.std():.2f}, min {fz.min():.2f}, max {fz.max():.2f}, in contact {100 * duty:.0f} percent of '
              f'the samples; peak since the approach {(-gt_c[:, 2]).max():.1f} N', flush=True)
        held = abs(fz.mean() - args.target_n) <= args.force_tol_n and fz.std() <= args.force_std_n
        print(f'      steady {args.target_n:.0f} N within {args.force_tol_n} N and std <= {args.force_std_n} N: '
              f'{"REACHED" if held else "NOT REACHED (the contact bounces, see docs/sim_gazebo.md)"}', flush=True)
        if args.require_hold:
            check('steady contact force at the target (ground truth)', held)
        if args.csv:
            tc, vc, gt = align(node.cond_log, node.truth_log, contact_from, node.sim_time)
            np.savetxt(args.csv, np.hstack([tc[:, None], vc, gt]), delimiter=',',
                       header='t,cF_x,cF_y,cF_z,cT_x,cT_y,cT_z,gF_x,gF_y,gF_z,gT_x,gT_y,gT_z')

        # 5. release: target to zero, disengage, retreat
        node.target(0.0)
        node.sim_wait(3.0)
        ok = node.set_jtac(admittance__selected_axes=[False] * 6)
        node.sim_wait(1.0)
        node.send(A, 8.0)
        node.sim_wait(1.5)
        _, gt_f = window(node.truth_log, node.sim_time - 1.0, node.sim_time)
        check('disengaged (parameters set live)', ok)
        check('force released after the retreat (ground truth ~ 0)', np.abs(gt_f[:, :3]).max() < 0.5,
              f'max|F| {np.abs(gt_f[:, :3]).max():.3f} N, tcp x {node.pose("tcp_link")[1][0]:.3f} m')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if proc:
            st.stop(proc)
    print('CONTACT PASS' if all(results) else 'CONTACT FAIL', flush=True)
    return 0 if all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
