#!/usr/bin/env python3
"""Obstacle stop from the simulated lidar (run_lidar_stop.sh starts everything).

A box stands on the floor in the straight path between the staging pose and the first stand. The navigator is told
about it only through /scan (`--mode with`) or not at all (`--mode without`).

  1. drive to the staging pose (three legs around the box), plan_stands for the smoke wall from there (as stand_navigation_test.py);
  2. with the lidar: check the scan against the box's true geometry (range along the bearing to its centre);
  3. send the base to the first stand and track the true footprint (odom) against the box rectangle;
  4. with the lidar: expect the typed result OBSTACLE, no overlap and no contact, and report the footprint-to-box gap at
     the stop against the configured stop distance. Without: expect no OBSTACLE result and the base against the box
     (footprint touching or overlapping it in odom, or MuJoCo contacts between a base geom and the box in the sim log).

Exit code 0 when the expectation of the mode holds.
"""
import argparse
import math
import os
import re
import sys
import threading
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from reachability_msgs.srv import PlanStands
from sensor_msgs.msg import LaserScan
from stand_navigator.core import arm_to_footprint, load_profile
from stand_navigator_msgs.action import NavigateToStand

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import stand_navigation_test as snt  # noqa: E402

LIDAR_MOUNT_X = 0.45   # run_lidar_stop.sh uses the launch default pose 0.45 0 0.2


def rect(cx, cy, sx, sy):
    return [(cx - sx / 2, cy - sy / 2), (cx + sx / 2, cy - sy / 2), (cx + sx / 2, cy + sy / 2), (cx - sx / 2, cy + sy / 2)]


def footprint_world(fp, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in fp]


def _seg_dist(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / (ax * ax + ay * ay)))
    return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)


def polygon_gap(a, b):
    """Distance between two convex polygons, negative (minus the least overlap along a separating axis) when they overlap."""
    least = math.inf
    for poly in (a, b):
        for i in range(len(poly)):
            p, q = poly[i], poly[(i + 1) % len(poly)]
            nx, ny = q[1] - p[1], p[0] - q[0]
            n = math.hypot(nx, ny)
            nx, ny = nx / n, ny / n
            pa = [nx * x + ny * y for x, y in a]
            pb = [nx * x + ny * y for x, y in b]
            least = min(least, min(max(pa) - min(pb), max(pb) - min(pa)))   # < 0: a gap along this axis
    if least > 0.0:
        return -least
    return min(_seg_dist(p, poly[i], poly[(i + 1) % len(poly)])
               for pts, poly in ((a, b), (b, a)) for p in pts for i in range(len(poly)))


def ray_box(origin, angle, box):
    """Range from `origin` along `angle` (world) to the axis-aligned rectangle `rect(*box)`, inf for a miss."""
    (x0, y0), (x1, y1) = rect(*box)[0], rect(*box)[2]
    d = (math.cos(angle), math.sin(angle))
    t0, t1 = 0.0, math.inf
    for o, v, lo, hi in ((origin[0], d[0], x0, x1), (origin[1], d[1], y0, y1)):
        if abs(v) < 1e-12:
            if not lo <= o <= hi:
                return math.inf
            continue
        ta, tb = sorted(((lo - o) / v, (hi - o) / v))
        t0, t1 = max(t0, ta), min(t1, tb)
    return t0 if t0 <= t1 else math.inf


class Run(snt.T):
    def __init__(self):
        super().__init__()
        self.scan = None
        self.scans = 0
        self.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        self.track = []   # (t, x, y, yaw)

    def _scan(self, m):
        with self.lock:
            self.scan = m
            self.scans += 1

    def sample(self):
        self.track.append((time.monotonic(), *self.pose()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', required=True)
    ap.add_argument('--mode', choices=('with', 'without'), required=True)
    ap.add_argument('--box', type=float, nargs=4, required=True, metavar=('CX', 'CY', 'SX', 'SY'))
    ap.add_argument('--stop-m', type=float, default=0.30, help='obstacle_stop_m of the navigator')
    ap.add_argument('--sim-log', default='')
    ap.add_argument('--timeout', type=float, default=90.0)
    args = ap.parse_args()
    prof = load_profile(args.profile)
    rclpy.init()
    node = Run()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    failures = []
    lidar = args.mode == 'with'
    box_poly = rect(*args.box)

    def check(name, ok, detail):
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}", flush=True)
        if not ok:
            failures.append(name)

    def finish():
        print(f'LIDAR STOP ({args.mode} lidar)', 'FAILED: ' + ', '.join(failures) if failures else 'PASSED', flush=True)
        os._exit(1 if failures else 0)

    t0 = time.monotonic()
    while time.monotonic() - t0 < args.timeout:
        odom, js, raw = node.state()
        if (odom is not None and js is not None and raw is not None and 'arm_elbow_joint' in js
                and node.nav.server_is_ready() and node.plan.service_is_ready() and (node.scans > 0 or not lidar)):
            break
        time.sleep(0.2)
    else:
        odom, js, raw = node.state()
        print(f'[FAIL] startup: odom {odom is not None}, joint_states {js is not None}, ft_raw {raw is not None}, '
              f'navigate_to_stand {node.nav.server_is_ready()}, plan_stands {node.plan.service_is_ready()}, '
              f'scans {node.scans}', flush=True)
        os._exit(1)
    time.sleep(2.0)
    node.arm0 = {k: v for k, v in node.state()[1].items() if k.startswith(('arm_', 'lift_'))}
    node.f0 = node.state()[2]

    # 1. staging pose and stands: three legs that keep clear of the box (back, down while turning, across), ending
    # 3 m below the stand facing +y, so the straight way to the stand is straight ahead of the lidar
    for leg in ((-1.5, 0.0, 0.0), (-1.5, -3.0, 1.571), (0.18, -3.0, 1.571)):
        r = node.drive(*leg)
        check(f'staging leg {leg}', r is not None and r.error_code == 0,
              'no result' if r is None else f'code {r.error_code} {r.error_msg}')
        if r is None or r.error_code != 0:
            finish()
    time.sleep(1.0)
    mount = prof.arm_mount
    fx, fy, fyaw = node.pose()
    c, s = math.cos(fyaw), math.sin(fyaw)
    req = PlanStands.Request()
    req.targets_world = snt.wall_targets()
    req.wall_point.x = snt.WALL_X
    req.wall_normal.x = -1.0
    req.lift_min, req.lift_max, req.lift_step = 0.0, 1.5, 0.1
    req.arm_mount_offset_z = 1.17
    req.has_start, req.start_x, req.start_y, req.start_yaw = (
        True, fx + c * mount[0] - s * mount[1], fy + s * mount[0] + c * mount[1], fyaw)
    fut = node.plan.call_async(req)
    t1 = time.monotonic()
    while not fut.done() and time.monotonic() - t1 < 60.0:
        time.sleep(0.05)
    res = fut.result() if fut.done() else None
    check('plan_stands', res is not None and res.success and len(res.stands) >= 1,
          'no reply' if res is None else f'{len(res.stands)} stand(s)')
    if res is None or not res.stands:
        finish()
    st = res.stands[0]
    gx, gy, gyaw = arm_to_footprint(st.x, st.y, st.yaw, mount)
    print(f'first stand: arm base ({st.x:.3f}, {st.y:.3f}) yaw {st.yaw:.3f}; base_footprint goal ({gx:.3f}, {gy:.3f}) '
          f'yaw {gyaw:.3f}; staging footprint ({fx:.3f}, {fy:.3f}) yaw {fyaw:.3f}', flush=True)

    # is the box in the straight path? sweep the footprint (start yaw to goal yaw) along the line
    fp = _footprint(args.profile)
    swept = min(polygon_gap(footprint_world(fp, (fx + (gx - fx) * k / 50, fy + (gy - fy) * k / 50,
                                                 fyaw + (gyaw - fyaw) * k / 50)), box_poly) for k in range(51))
    print(f'box {args.box}: swept footprint gap along the straight line {swept:.3f} m (negative: the line runs into it)',
          flush=True)
    check('box is in the straight path', swept < 0.0, f'{swept:.3f} m')

    # 2. the scan against the box's true geometry (from the staging pose)
    if lidar:
        time.sleep(0.5)
        with node.lock:
            scan = node.scan
        c, s = math.cos(fyaw), math.sin(fyaw)
        origin = (fx + c * LIDAR_MOUNT_X, fy + s * LIDAR_MOUNT_X)
        bearing = math.atan2(args.box[1] - origin[1], args.box[0] - origin[0])
        want = ray_box(origin, bearing, args.box)
        k = round((bearing - fyaw - scan.angle_min) / scan.angle_increment)
        got = scan.ranges[k] if 0 <= k < len(scan.ranges) else math.nan
        check('scan sees the box', math.isfinite(got) and abs(got - want) < 0.03,
              f'ray {k} ({math.degrees(scan.angle_min + k * scan.angle_increment):.1f} deg): scan {got:.3f} m, '
              f'geometry {want:.3f} m; {len(scan.ranges)} rays, frame {scan.header.frame_id}, '
              f'range {scan.range_min:g}..{scan.range_max:g} m')

    # 3. drive to the first stand
    t_drive = time.monotonic()
    r = node.drive(st.x, st.y, st.yaw, timeout=60.0)
    took = time.monotonic() - t_drive
    time.sleep(1.5)
    node.sample()
    pose = node.pose()
    fp_now = footprint_world(fp, pose)
    gap = polygon_gap(fp_now, box_poly)
    gaps = [polygon_gap(footprint_world(fp, p[1:]), box_poly) for p in node.track]
    min_gap = min(gaps) if gaps else math.nan
    code = None if r is None else r.error_code
    print(f'drive result: {"no result" if r is None else f"code {r.error_code} {r.error_msg}"}; took {took:.1f} s; '
          f'final footprint pose ({pose[0]:.3f}, {pose[1]:.3f}) yaw {pose[2]:.3f}; '
          f'distance from the goal {math.hypot(pose[0] - gx, pose[1] - gy):.3f} m', flush=True)
    print(f'footprint to box gap: final {gap:.3f} m, smallest over the drive {min_gap:.3f} m '
          f'(negative: footprint overlaps the box)', flush=True)
    contacts = []
    if args.sim_log and os.path.exists(args.sim_log):
        with open(args.sim_log, errors='replace') as f:
            contacts = [ln.strip() for ln in f if 'UNEXPECTED_CONTACT' in ln and 'obstacle_0' in ln]
    print(f'sim log: {len(contacts)} unexpected-contact line(s) against obstacle_0'
          + (f', first: {re.sub(r"^.*UNEXPECTED_CONTACT", "UNEXPECTED_CONTACT", contacts[0])}' if contacts else ''),
          flush=True)

    if lidar:
        check('typed OBSTACLE result', code == NavigateToStand.Result.OBSTACLE,
              f'code {code} (OBSTACLE is {NavigateToStand.Result.OBSTACLE})')
        check('no overlap with the box over the drive', min_gap > 0.0, f'smallest gap {min_gap:.3f} m')
        check('no contact in the sim log', not contacts, f'{len(contacts)} line(s)')
        # the navigator stops at obstacle_stop_m + v^2 / (2 a) ahead of the lidar, which sits on the front edge
        check('stops short of the configured stop distance', gap >= args.stop_m - 0.05,
              f'gap {gap:.3f} m, obstacle_stop_m {args.stop_m:.2f} m (+ braking term v^2 / 2a, up to 0.25 m at '
              f'0.5 m/s; 0.05 m allowed for scan age)')
    else:
        check('no OBSTACLE result (the stack cannot see the box)', code != NavigateToStand.Result.OBSTACLE,
              f'code {code}')
        check('base reached the box', min_gap <= 0.005 or bool(contacts),
              f'smallest footprint gap {min_gap:.3f} m, {len(contacts)} contact line(s)')
    finish()


def _footprint(profile_path):
    import yaml
    with open(profile_path) as f:
        return yaml.safe_load(f)['base']['footprint']


if __name__ == '__main__':
    main()
