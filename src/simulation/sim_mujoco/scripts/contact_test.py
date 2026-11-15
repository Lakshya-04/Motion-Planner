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

import json

import rclpy
from geometry_msgs.msg import Wrench, WrenchStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

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
        self.run_rows = []   # (t_s, x, y, z, |Fz|) of the whole run, tool point in base_footprint
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(WrenchStamped, '/ft_conditioner/wrench_conditioned', self._cond,
                                 qos_profile_sensor_data)
        self.create_subscription(WrenchStamped, '/sim_mujoco/ft_ground_truth', self._truth_stamped,
                                 qos_profile_sensor_data)
        self.target_pub = self.create_publisher(
            Wrench, '/ft_bridge_hardware_ft_sensor_tcp/target_wrench',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.target_via = 'bridge'   # or 'jtac': JTAC's own ~/target_force_n, the bridge target stays 0
        self.jtac_target_pub = self.create_publisher(Float64, f'/{CONTROLLER}/target_force_n', 10)
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
        pos = None
        try:
            tf = self.tf_buffer.lookup_transform('base_footprint', 'tcp_link', rclpy.time.Time())
            pos = (tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z)
        except Exception:  # noqa: BLE001  (no transform yet: skip the row)
            pass
        with self.lock:
            if self.record:
                self.truth_by_stamp[self._stamp(m)] = self._w(m)
            if pos is not None:
                self.run_rows.append((self._stamp(m) * 1e-9, *pos, abs(self._w(m)[2])))

    def write_recording(self, path, wall_x, wall_range_y, wall_range_z):
        """The whole run as the score_run CSV (t,x,y,z,force,active) plus a wall JSON beside it. The wall is
        the plane x = wall_x in base_footprint: u along y, v along z, normal u x v = +x into the wall."""
        with self.lock:
            rows = list(self.run_rows)
        t0 = rows[0][0] if rows else 0.0
        with open(path, 'w') as f:
            f.write('t,x,y,z,force,active\n')
            for t, x, y, z, fz in rows:
                f.write(f'{t - t0:.4f},{x:.5f},{y:.5f},{z:.5f},{fz:.4f},0\n')
        wall = {'origin': [wall_x, 0.0, 0.0], 'u': [0, 1, 0], 'v': [0, 0, 1],
                'u_range': list(wall_range_y), 'v_range': list(wall_range_z)}
        with open(path + '.wall.json', 'w') as f:
            json.dump(wall, f)
        return len(rows)

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
        if self.target_via == 'jtac':
            # JTAC adds its own target to the wrench it reads, so it sees the absolute force
            m = Float64()
            m.data = float(fz_push)
            self.jtac_target_pub.publish(m)
            w = Wrench()
            self.target_pub.publish(w)
            return
        w = Wrench()
        w.force.z = -float(fz_push)
        self.target_pub.publish(w)


AXES_OFF = [False] * 6


def release(node):
    """Leave the sim as found: tool-Z axis off and no target wrench latched in the bridge. Each step is tried
    even if the other fails (the sim may be gone), so an aborted run does not leave JTAC pushing."""
    for step in (lambda: node.set_axes(AXES_OFF), lambda: node.set_target(0.0)):
        try:
            step()
        except Exception as e:  # noqa: BLE001
            print(f'[WARN] cleanup step failed: {e}', flush=True)


def guarded(node, body):
    """Run body(); whatever happens, release the contact afterwards. Returns body's value."""
    try:
        return body()
    finally:
        release(node)


def release_time(trace, t_start, quiet=0.5, limit=0.5):
    """Seconds after t_start at which |Fz| left `limit` for good: the time of the last sample at or above it,
    provided the trace then stays below it for at least `quiet` s (a single low sample is a bounce, not a
    release). None when the force is still there or the quiet window is too short."""
    if not trace:
        return None
    high = [t for t, f in trace if abs(f) >= limit]
    if not high:
        return 0.0
    if trace[-1][0] - high[-1] < quiet:
        return None
    return high[-1] - t_start


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
    ap.add_argument('--err-force-max', type=float, default=0.05,
                    help='largest |conditioned - truth| force over the hold, N (noise-free sim: ~0.004 N seen)')
    ap.add_argument('--err-torque-max', type=float, default=0.02,
                    help='largest |conditioned - truth| torque over the hold, N m (~0.001 N m seen)')
    ap.add_argument('--no-calibrate', action='store_true', help='use the launch payload parameters')
    ap.add_argument('--tcp-z', type=float, default=0.165)
    ap.add_argument('--timeout', type=float, default=120.0)
    ap.add_argument('--target-path', choices=['bridge', 'jtac'], default='bridge',
                    help="where the contact target is carried: the F/T bridge (default) or JTAC's own ~/target_force_n")
    ap.add_argument('--record', default='', help='write the run as a score_run CSV (and PATH.wall.json)')
    args = ap.parse_args()
    rclpy.init()
    node = ContactTest()
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
                and all(c.service_is_ready() for c in node.srv.values())):
            break
        time.sleep(0.2)
    else:
        print('[FAIL] startup: missing joint_states / raw / conditioned wrench / action server / services')
        os._exit(1)

    def run():
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
                    if k.strip() not in ('payload_mass_kg', 'payload_cog_xyz', 'bias'):
                        continue
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
            node.hold(1.0)
            cond_rest = []
            for _ in range(25):                  # averaged over 0.5 s, not one sample
                cond_rest.append(node.cond_now())
                time.sleep(0.02)
            cond_rest = mean(cond_rest)
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
        check('conditioned force error bounded (max)', max(stats[n][4] for n in NAMES[:3]) < args.err_force_max,
              f'worst max {max(stats[n][4] for n in NAMES[:3]):.4f} N, limit {args.err_force_max} N')
        check('conditioned torque error bounded (max)', max(stats[n][4] for n in NAMES[3:]) < args.err_torque_max,
              f'worst max {max(stats[n][4] for n in NAMES[3:]):.4f} N m, limit {args.err_torque_max} N m')

        # ---- 4. disengage and retreat ----------------------------------------------------------------
        check('disengage', node.set_axes([False] * 6), 'selected_axes all false')
        node.set_target(0.0)
        trace, stop = [], threading.Event()

        def sample():
            while not stop.is_set():
                trace.append((time.monotonic(), node.state()[2][2]))
                time.sleep(0.02)

        threading.Thread(target=sample, daemon=True).start()
        t_start = time.monotonic()
        code = node.move(FREE, 4.0)
        time.sleep(1.5)                  # quiet window after the trajectory ends
        stop.set()
        t_released = release_time(trace, t_start)
        check('retreat trajectory finished', code == 0, f'error_code {code}')
        check('force released after disengage', t_released is not None,
              f'truth |Fz| below 0.5 N for good {t_released:.1f} s after the retreat started '
              f'(trajectory takes 4 s)' if t_released is not None
              else f'truth Fz = {node.state()[2][2]:.2f} N 1.5 s after the retreat ended')
        time.sleep(1.0)
        final = mean([s[2] for s in node.hold(1.0)])
        check('stays released', abs(final[2]) < 0.5, f'truth Fz = {final[2]:.3f} N')


    guarded(node, run)

    if args.record:
        n = node.write_recording(args.record, 0.98, (0.0, 0.3), (1.0, 1.3))
        print(f'recorded {n} samples to {args.record}', flush=True)
    print('CONTACT TEST', 'FAILED: ' + ', '.join(failures) if failures else 'PASSED', flush=True)
    os._exit(1 if failures else 0)


if __name__ == '__main__':
    main()
