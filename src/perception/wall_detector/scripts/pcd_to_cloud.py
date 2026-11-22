#!/usr/bin/env python3
"""Publish an ASCII .pcd (x y z [label]) as a sensor_msgs/PointCloud2 at a fixed rate, for wall_detector.

    pcd_to_cloud.py FILE.pcd [--topic /depth_sensor/points] [--frame odom] [--rate 5] [--max-points 200000]

The cloud is static (a stand-in for a sensor that sees the whole wall); it is sent best-effort like a sensor, as the
detector subscribes with SensorDataQoS. Stops on Ctrl-C / SIGINT.
"""
import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


def read_pcd_xyz(path):
    with open(path) as f:
        lines = f.read().splitlines()
    start = next(i for i, ln in enumerate(lines) if ln.startswith('DATA')) + 1
    return np.array([[float(t) for t in ln.split()[:3]] for ln in lines[start:] if ln.strip()], dtype=np.float32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd')
    ap.add_argument('--topic', default='/depth_sensor/points')
    ap.add_argument('--frame', default='odom')
    ap.add_argument('--rate', type=float, default=5.0)
    ap.add_argument('--max-points', type=int, default=200000)
    a = ap.parse_args()
    pts = read_pcd_xyz(a.pcd)
    if len(pts) > a.max_points:
        pts = pts[np.random.default_rng(1).choice(len(pts), a.max_points, replace=False)]
    rclpy.init()
    node = Node('pcd_to_cloud')
    pub = node.create_publisher(PointCloud2, a.topic, qos_profile_sensor_data)
    msg = PointCloud2()
    msg.header.frame_id = a.frame
    msg.height, msg.width = 1, len(pts)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
    msg.is_bigendian, msg.point_step, msg.row_step, msg.is_dense = False, 12, 12 * len(pts), True
    msg.data = pts.tobytes()

    def tick():
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)

    node.create_timer(1.0 / a.rate, tick)
    node.get_logger().info(f'publishing {len(pts)} points of {a.pcd} on {a.topic} in {a.frame} at {a.rate} Hz')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
