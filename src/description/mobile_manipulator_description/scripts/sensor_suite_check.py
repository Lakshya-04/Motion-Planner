#!/usr/bin/env python3
"""Check a running simulator against a platform's sensor suite: scan / depth frame, scan geometry, rate, mount TF.

    sensor_suite_check.py ranger_mini [--seconds 15] [--no-depth]

Start the sim first with platform:=<same> lidar:=true depth_sensor:=true. Rates are measured on the message
stamps (sim time under Gazebo) and on the wall clock; the stamp rate is the one that is checked.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from mobile_manipulator_description.sensor_suite import suite_values


def _stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Probe(Node):
    def __init__(self, with_depth):
        super().__init__('sensor_suite_check')
        self.scan, self.cloud = [], []   # (stamp, wall)
        self.first_scan = None
        self.first_cloud_frame = None
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        if with_depth:
            self.create_subscription(PointCloud2, '/depth_sensor/points', self._on_cloud, qos_profile_sensor_data)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

    def _on_scan(self, msg):
        self.first_scan = self.first_scan or msg
        self.scan.append((_stamp(msg), time.monotonic()))

    def _on_cloud(self, msg):
        self.first_cloud_frame = self.first_cloud_frame or msg.header.frame_id
        self.cloud.append((_stamp(msg), time.monotonic()))


def _rates(samples):
    if len(samples) < 3:
        return float('nan'), float('nan')
    n = len(samples) - 1
    return n / (samples[-1][0] - samples[0][0]), n / (samples[-1][1] - samples[0][1])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('platform')
    ap.add_argument('--seconds', type=float, default=15.0)
    ap.add_argument('--no-depth', action='store_true')
    ap.add_argument('--rate-tol', type=float, default=0.15, help='allowed relative rate error')
    args = ap.parse_args(argv)
    want = suite_values(args.platform)
    rclpy.init()
    node = Probe(not args.no_depth)
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    ok = True

    def check(label, good, detail):
        nonlocal ok
        ok = ok and good
        print(f'{"ok  " if good else "FAIL"} {label}: {detail}')

    def mount(frame, pose):
        x, y, z = [float(v) for v in pose.split()[:3]]
        try:
            t = node.tf.lookup_transform('base_footprint', frame, rclpy.time.Time()).transform.translation
        except TransformException as e:
            check(f'tf base_footprint -> {frame}', False, str(e).splitlines()[0])
            return
        err = math.dist((t.x, t.y, t.z), (x, y, z))
        check(f'tf base_footprint -> {frame}', err < 1e-3, f'({t.x:.3f} {t.y:.3f} {t.z:.3f}) wanted ({x} {y} {z})')

    msg = node.first_scan
    check('/scan received', msg is not None, f'{len(node.scan)} scans')
    if msg is not None:
        rays = len(msg.ranges)
        span = float(want['lidar_angle_max_deg']) - float(want['lidar_angle_min_deg'])
        inc = float(want['lidar_angle_increment_deg'])
        check('scan frame', msg.header.frame_id == want['lidar_frame'], f'{msg.header.frame_id} wanted {want["lidar_frame"]}')
        check('scan angles', abs(math.degrees(msg.angle_min) - float(want['lidar_angle_min_deg'])) < 0.01
              and abs(math.degrees(msg.angle_increment) - inc) < 0.01 * max(inc, 1.0),
              f'{math.degrees(msg.angle_min):.3f} deg, step {math.degrees(msg.angle_increment):.4f}, {rays} rays; '
              f'wanted {want["lidar_angle_min_deg"]}, step {inc}, about {int(span / inc) + 1} rays')
        check('scan range', abs(msg.range_min - float(want['lidar_range_min_m'])) < 1e-3
              and abs(msg.range_max - float(want['lidar_range_max_m'])) < 1e-3,
              f'{msg.range_min:.3f} to {msg.range_max:.3f} m, wanted {want["lidar_range_min_m"]} to {want["lidar_range_max_m"]}')
        stamp_hz, wall_hz = _rates(node.scan)
        wanted = float(want['lidar_rate_hz'])
        check('scan rate', abs(stamp_hz - wanted) <= args.rate_tol * wanted,
              f'{stamp_hz:.2f} Hz on stamps ({wall_hz:.2f} Hz wall), wanted {wanted}')
        mount(want['lidar_frame'], want['lidar_pose'])
    if not args.no_depth:
        check('/depth_sensor/points received', bool(node.cloud), f'{len(node.cloud)} clouds')
        if node.cloud:
            check('depth frame', node.first_cloud_frame == want['depth_sensor_frame'],
                  f'{node.first_cloud_frame} wanted {want["depth_sensor_frame"]}')
            stamp_hz, wall_hz = _rates(node.cloud)
            wanted = float(want['depth_sensor_rate_hz'])
            check('depth rate', abs(stamp_hz - wanted) <= args.rate_tol * wanted,
                  f'{stamp_hz:.2f} Hz on stamps ({wall_hz:.2f} Hz wall), wanted {wanted}')
            mount(want['depth_sensor_frame'], want['depth_sensor_pose'])
    node.destroy_node()
    rclpy.shutdown()
    print('SUITE-CHECK ' + ('PASS' if ok else 'FAIL'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
