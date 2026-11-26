#!/usr/bin/env python3
"""What robot_self_filter removes from the simulated depth cloud (run_self_filter_check.sh starts everything).

Pairs the raw cloud (/depth_sensor/points) with the filtered one (/depth_sensor/points_filtered) by stamp and reports,
averaged over --pairs: the finite points raw and filtered, how many points the filter removed, how many of those
lie on the wall (should be none) and where the removed ones are in base_footprint. With --scene-raw and
--scene-filtered (wall_detector wall_scene.json from a detector on each cloud) it also prints how far apart the two
fitted planes are.
"""
import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener


def xyz(msg):
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
    p = raw[:, :12].copy().view('<f4').reshape(-1, 3).astype(float)
    return p[np.isfinite(p).all(axis=1)]


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Pairs(Node):
    def __init__(self):
        super().__init__('self_filter_check')
        self.raw, self.pairs = {}, []
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.create_subscription(PointCloud2, '/depth_sensor/points', self._raw, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/depth_sensor/points_filtered', self._filtered, qos_profile_sensor_data)

    def _raw(self, m):
        self.raw[(m.header.stamp.sec, m.header.stamp.nanosec)] = m
        for k in sorted(self.raw)[:-10]:
            del self.raw[k]

    def _filtered(self, m):
        r = self.raw.get((m.header.stamp.sec, m.header.stamp.nanosec))
        if r is not None:
            self.pairs.append((r, m))


def plane_diff(a, b):
    wa, wb = a['walls'][0], b['walls'][0]
    na, nb = np.array(wa['normal_outward']), np.array(wb['normal_outward'])
    ang = math.degrees(math.acos(max(-1.0, min(1.0, float(na @ nb)))))
    da, db = wa['plane_abcd'][3], wb['plane_abcd'][3]
    return (ang, (da - db) * 1000.0, wa['num_inliers'], wb['num_inliers'], wa['plane_rms_mm'], wb['plane_rms_mm'])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--wall-x', type=float, default=0.815, help='wall distance along the sensor x axis, m')
    ap.add_argument('--wall-tol', type=float, default=0.02)
    ap.add_argument('--pairs', type=int, default=5)
    ap.add_argument('--scene-raw')
    ap.add_argument('--scene-filtered')
    ap.add_argument('--label', default='')
    args = ap.parse_args()
    rclpy.init()
    node = Pairs()
    end = time.time() + 90
    while len(node.pairs) < args.pairs and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(node.pairs) < args.pairs:
        print('FAIL  no raw/filtered cloud pairs (is the self filter running, does it keep the stamp?)')
        return 1
    rows = []
    for raw_msg, filt_msg in node.pairs[-args.pairs:]:
        pr, pf = xyz(raw_msg), xyz(filt_msg)
        kept = {row.tobytes() for row in pf.astype('<f4')}
        removed = np.array([not (row.tobytes() in kept) for row in pr.astype('<f4')])
        wall = np.abs(pr[:, 0] - args.wall_x) < args.wall_tol
        rows.append((len(pr), len(pf), int(removed.sum()), int((removed & wall).sum()), int(wall.sum()),
                     pr[removed], raw_msg.header.frame_id))
    frame = rows[0][6]
    removed_pts = np.concatenate([r[5] for r in rows]) if any(len(r[5]) for r in rows) else np.zeros((0, 3))
    m = np.mean([r[:5] for r in rows], axis=0)
    tag = f' [{args.label}]' if args.label else ''
    print(f'self filter{tag}: raw {m[0]:.0f} finite points, filtered {m[1]:.0f}, removed {m[2]:.0f}, '
          f'wall points (|x - {args.wall_x}| < {args.wall_tol}) {m[4]:.0f} of which removed {m[3]:.0f}, '
          f'non-wall raw points {m[0] - m[4]:.0f}', flush=True)
    if len(removed_pts):
        try:
            t = node.tf.lookup_transform('base_footprint', frame, rclpy.time.Time())
            b = removed_pts @ quat_matrix(t.transform.rotation).T + np.array(
                [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
            lo, hi = b.min(axis=0), b.max(axis=0)
            print(f'  removed points in base_footprint: x {lo[0]:.2f}..{hi[0]:.2f}, y {lo[1]:.2f}..{hi[1]:.2f}, '
                  f'z {lo[2]:.2f}..{hi[2]:.2f} m', flush=True)
        except Exception as e:  # tf2 raises several exception types
            print(f'  no TF {frame} -> base_footprint: {e!r}')
    if args.scene_raw and args.scene_filtered:
        d = plane_diff(json.load(open(args.scene_raw)), json.load(open(args.scene_filtered)))
        print(f'  wall fit raw vs filtered: normal differs {d[0]:.4f} deg, plane offset differs {d[1]:.2f} mm, '
              f'inliers {d[2]} vs {d[3]}, rms {d[4]:.2f} vs {d[5]:.2f} mm', flush=True)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
