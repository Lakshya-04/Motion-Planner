#!/usr/bin/env python3
"""Contact-controlled test for the MuJoCo backend: the full force path in simulation.

  /sim_mujoco/ft_raw  ->  ft_conditioner  ->  ft_bridge_hardware (sensor ft_sensor_tcp)  ->  JTAC

Run against a launched sim: `controller:=jtac conditioner:=true`, UR10e profile, smoke wall scene
(scripts/run_contact.sh does that). Steps:

  1. calibrate the conditioner with its own services (capture_pose in several free-space wrist poses,
     identify) and compare the fit with the known tool inertial (0.5 kg at the sensor origin, no bias);
  2. approach the wall, engage the tool-Z admittance axis with a target contact force through the
     bridge's ~/target_wrench, press, and hold;
  3. compare the steady contact force with the ground truth (/sim_mujoco/ft_ground_truth) and report the
     conditioned-vs-truth error statistics for force and torque over the hold;
  4. disengage, retreat, and check the force is released (and how long that takes).

Exit code 0 when every check passes. Output is one line per check plus the statistics.
"""
import argparse
import math
import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Wrench, WrenchStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from smoke_test import FREE, HOME, PRESS, Smoke, mean  # noqa: E402

CONTROLLER = 'combined_arm_lift_controller'
TOOL_MASS = 0.5          # generic_tool inertial; its centre of gravity is the sensor origin
# wrist poses for the payload identification, all in free space (tool pointing away from the wall)
CAL_POSES = [
    HOME,
    HOME[:4] + [HOME[4] + 0.8] + HOME[5:],
    HOME[:4] + [HOME[4] - 0.8] + HOME[5:],
    HOME[:5] + [HOME[5] + 0.8, 0.0],
    HOME[:5] + [HOME[5] - 0.8, 1.0],
    HOME[:3] + [HOME[3] - 0.7, HOME[4] + 0.6, HOME[5] + 0.5, 0.8],
]
NAMES = ['fx', 'fy', 'fz', 'tx', 'ty', 'tz']


class ContactTest(Smoke):
    def __init__(self):
        super().__init__(CONTROLLER)
        self.cond = {}       # stamp (ns) -> conditioned wrench (tcp frame)
        self.truth_by_stamp = {}
        self.cond_last = None
        self.record = False
        self.create_subscription(WrenchStamped, '/ft_conditioner/wrench_conditioned', self._cond,
                                 qos_profile_sensor_data)
        self.create_subscription(WrenchStamped, '/sim_mujoco/ft_ground_truth', self._truth_stamped,
                                 qos_profile_sensor_data)
        self.target_pub = self.create_publisher(
            Wrench, '/ft_bridge_hardware_ft_sensor_tcp/target_wrench',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.params = self.create_client(SetParameters, f'/{CONTROLLER}/set_parameters')
        self.srv = {k: self.create_client(Trigger, f'/ft_conditioner/{k}')
                    for k in ('capture_pose', 'identify', 'clear_samples')}

    @staticmethod
    def _stamp(m):
        return m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec

    def _cond(self, m):
        with self.lock:
            self.cond_last = self._w(m)
            if self.record:
                self.cond[self._stamp(m)] = self._w(m)

    def _truth_stamped(self, m):
        with self.lock:
            if self.record:
                self.truth_by_stamp[self._stamp(m)] = self._w(m)

    def cond_now(self):
        with self.lock:
            return self.cond_last

    def call(self, client, req, timeout=10.0):
        fut = client.call_async(req)
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                raise RuntimeError('service call timed out')
            time.sleep(0.01)
        return fut.result()

    def trigger(self, name):
        return self.call(self.srv[name], Trigger.Request())

    def set_axes(self, axes):
        p = Parameter()
        p.name = 'admittance.selected_axes'
        p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL_ARRAY, bool_array_value=list(axes))
        req = SetParameters.Request()
        req.parameters = [p]
        res = self.call(self.params, req)
        return all(r.successful for r in res.results)

    def set_target(self, fz_push):
        """Contact target through the bridge: it subtracts this wrench, so the admittance error is zero
        when the wall pushes back with fz = -fz_push (pressing along the tool +z)."""
        w = Wrench()
        w.force.z = -float(fz_push)
        self.target_pub.publish(w)


def rms(v):
    return math.sqrt(sum(x * x for x in v) / len(v)) if v else float('nan')


def std(v):
    m = sum(v) / len(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / len(v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--target', type=float, default=10.0, help='contact force target, N')
    ap.add_argument('--ref-frac', type=float, default=0.375,
                    help='commanded reference, as a fraction of the joint-space way from FREE (3 cm short of '
                         'the wall) to PRESS (5 cm past): 0.375 puts the reference on the wall surface. '
                         'The admittance spring pulls toward the reference, so the steady force is '
                         'target + K * (reference depth past the surface)')
    ap.add_argument('--hold', type=float, default=10.0, help='seconds of contact to record')
    ap.add_argument('--tol-abs', type=float, default=1.5, help='steady force tolerance, N')
    ap.add_argument('--tol-rel', type=float, default=0.15, help='or this fraction of the target')
    ap.add_argument('--no-calibrate', action='store_true', help='use the launch payload parameters')
    ap.add_argument('--tcp-z', type=float, default=0.165)
    ap.add_argument('--timeout', type=float, default=120.0)
    args = ap.parse_args()
    rclpy.init()
    node = ContactTest()
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
                and all(c.service_is_ready() for c in node.srv.values())):
            break
        time.sleep(0.2)
    else:
        print('[FAIL] startup: missing joint_states / raw / conditioned wrench / action server / services')
        os._exit(1)

    # ---- 1. calibration ------------------------------------------------------------------------
    if not args.no_calibrate:
        node.trigger('clear_samples')
        node.move(CAL_POSES[0], 4.0)
        node.hold(1.0)
        before = mean([s[1] for s in node.hold(0.5)])
        cond_before = node.cond_now()
        print(f'uncalibrated conditioner at home pose: |F_cond| = '
              f'{math.sqrt(sum(v * v for v in cond_before[:3])):.2f} N (payload not removed yet)')
        gdirs = []
        for i, pose in enumerate(CAL_POSES):
            node.move(pose, 5.0)
            samples = node.hold(1.5)
            truth_f = max(abs(v) for s in samples for v in s[2][:3])
            res = node.trigger('capture_pose')
            raw = mean([s[1] for s in node.hold(0.2)])
            norm = math.sqrt(sum(v * v for v in raw[:3]))
            gdirs.append([round(v / norm, 2) for v in raw[:3]])
            check(f'calibration pose {i + 1} free and captured', res.success and truth_f < 0.05,
                  f'max |truth F| {truth_f:.3f} N, gravity dir in sensor frame {gdirs[-1]}')
        res = node.trigger('identify')
        print(res.message)
        check('payload identification succeeded', res.success, res.message.splitlines()[0])
        if res.success:
            vals = {}
            for line in res.message.splitlines()[1:]:
                k, _, v = line.partition(':')
                vals[k.strip()] = [float(x) for x in v.strip(' []').replace(',', ' ').split()]
            mass = vals['payload_mass_kg'][0]
            cog = vals['payload_cog_xyz']
            bias = vals['bias']
            check('identified mass matches the tool inertial', abs(mass - TOOL_MASS) < 0.01 * TOOL_MASS + 0.005,
                  f'{mass:.4f} kg vs {TOOL_MASS} kg')
            check('identified centre of gravity at the sensor origin', max(abs(c) for c in cog) < 0.005,
                  f'{[round(c, 5) for c in cog]} m vs [0, 0, 0]')
            check('identified bias ~ 0 (sim sensor has no offset)', max(abs(b) for b in bias[:3]) < 0.05
                  and max(abs(b) for b in bias[3:]) < 0.02, f'{[round(b, 4) for b in bias]}')
        node.move(CAL_POSES[0], 5.0)
        node.hold(1.0)
        rest = [s[2] for s in node.hold(1.0)]
        cond_rest = mean([node.cond_now() for _ in range(1)])
        check('calibrated conditioner reads ~0 in free space', max(abs(v) for v in cond_rest) < 0.1,
              f'{[round(v, 3) for v in cond_rest]}')

    # ---- 2. approach, engage, press ------------------------------------------------------------
    node.move(FREE, 8.0)
    node.hold(1.0)
    free_cond = node.cond_now()
    check('free space after approach: conditioned wrench ~ 0', max(abs(v) for v in free_cond[:3]) < 0.1,
          f'F_cond = {[round(v, 3) for v in free_cond[:3]]} N, T_cond = {[round(v, 3) for v in free_cond[3:]]}')
    node.set_target(args.target)
    time.sleep(0.3)
    check('engage tool-Z admittance axis', node.set_axes([False, False, True, False, False, False]), 'set')
    ref = [a + (b - a) * args.ref_frac for a, b in zip(FREE, PRESS)]
    code = node.move(ref, 6.0)
    check('press trajectory finished', code == 0, f'error_code {code}')
    node.hold(4.0)                       # let the admittance settle
    with node.lock:
        node.record = True
        node.cond.clear()
        node.truth_by_stamp.clear()
    rows = node.hold(args.hold)
    with node.lock:
        node.record = False
        stamps = sorted(set(node.cond) & set(node.truth_by_stamp))
        pairs = [(node.cond[s], node.truth_by_stamp[s]) for s in stamps]
    truth_f = [s[2][2] for s in rows]
    steady = mean([s[2] for s in rows])
    tol = max(args.tol_abs, args.tol_rel * args.target)
    # ---- 3. steady force vs ground truth and conditioner accuracy -------------------------------
    check('steady contact force at the target (ground truth)', abs(-steady[2] - args.target) < tol,
          f'|Fz| = {-steady[2]:.2f} N (std {std(truth_f):.2f}) vs target {args.target:.1f} N, tolerance +-{tol:.2f} N')
    depth = (-steady[2] - args.target) / 300.0
    print(f'  implied reference depth past the wall: {1000 * depth:+.1f} mm '
          f'(steady force - target) / K, K = 300 N/m (jtac_controller.yaml, Z)')
    check('contact is along the tool axis', abs(steady[2]) > 3.0 * math.hypot(steady[0], steady[1]),
          f'truth F = {[round(v, 2) for v in steady[:3]]} N')
    check('enough synchronized samples', len(pairs) > 0.8 * args.hold * 100, f'{len(pairs)} pairs')
    print(f'conditioned vs ground truth over {len(pairs)} samples ({args.hold:.0f} s of contact), '
          f'error = conditioned - truth, both at tcp_link in the tool axes:')
    print(f'  {"":3s} {"truth mean":>11s} {"err mean":>9s} {"err std":>8s} {"err rms":>8s} {"err max":>8s}')
    stats = {}
    for k, name in enumerate(NAMES):
        err = [c[k] - t[k] for c, t in pairs]
        tm = sum(t[k] for _, t in pairs) / len(pairs)
        stats[name] = (tm, sum(err) / len(err), std(err), rms(err), max(abs(e) for e in err))
        unit = 'N' if k < 3 else 'Nm'
        print(f'  {name:3s} {tm:11.3f} {stats[name][1]:9.3f} {stats[name][2]:8.3f} {stats[name][3]:8.3f} '
              f'{stats[name][4]:8.3f}  {unit}', flush=True)
    f_tol = 0.05 * abs(steady[2]) + 0.3
    t_tol = 0.05 * abs(steady[2]) * args.tcp_z + 0.1
    check('conditioned force matches ground truth', max(stats[n][3] for n in NAMES[:3]) < f_tol,
          f'worst rms {max(stats[n][3] for n in NAMES[:3]):.3f} N, tolerance {f_tol:.2f} N')
    check('conditioned torque matches ground truth', max(stats[n][3] for n in NAMES[3:]) < t_tol,
          f'worst rms {max(stats[n][3] for n in NAMES[3:]):.3f} N m, tolerance {t_tol:.2f} N m')

    # ---- 4. disengage and retreat ----------------------------------------------------------------
    check('disengage', node.set_axes([False] * 6), 'selected_axes all false')
    node.set_target(0.0)
    t_start = time.monotonic()
    code = node.move(FREE, 4.0)
    t_released = None
    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline:
        if abs(node.state()[2][2]) < 0.5:
            t_released = time.monotonic() - t_start
            break
        time.sleep(0.05)
    check('retreat trajectory finished', code == 0, f'error_code {code}')
    check('force released after disengage', t_released is not None,
          f'truth |Fz| < 0.5 N {t_released:.1f} s after the retreat started'
          if t_released is not None else f'truth Fz = {node.state()[2][2]:.2f} N after 12 s')
    time.sleep(1.0)
    final = mean([s[2] for s in node.hold(1.0)])
    check('stays released', abs(final[2]) < 0.5, f'truth Fz = {final[2]:.3f} N')

    print('CONTACT TEST', 'FAILED: ' + ', '.join(failures) if failures else 'PASSED', flush=True)
    os._exit(1 if failures else 0)


if __name__ == '__main__':
    main()
