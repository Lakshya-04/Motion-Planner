"""Scan-to-scan matcher: /scan (LaserScan) -> /scan_odom (nav_msgs/Odometry) with a velocity only.

The twist is the motion between consecutive scans over their time difference, in the base frame. The pose
fields are left at zero with a huge covariance, so the filter fuses the twist and nothing else; a matcher
that drifts as a pose source would pull the filter with it.

Parameters: `scan_topic`, `output_topic`, `base_frame`, `odom_frame`, `max_dist_m` (pairing distance),
`max_rms_m` and `min_inlier_frac` (a match worse than that is dropped), `max_step_m` (so is a jump bigger than that),
`sigma_trans_m` and `sigma_rot_rad` (1 sigma of one match, turned into the twist covariance).
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from .scan_match import body_twist, match, scan_to_points, transform


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ScanMatch(Node):
    def __init__(self):
        super().__init__('scan_match')
        p = self.declare_parameter
        self.base_frame = p('base_frame', 'base_footprint').value
        self.odom_frame = p('odom_frame', 'odom').value
        self.max_dist = p('max_dist_m', 0.3).value
        self.max_rms = p('max_rms_m', 0.05).value
        self.min_inlier = p('min_inlier_frac', 0.5).value
        self.max_step = p('max_step_m', 0.5).value
        self.sigma_t = p('sigma_trans_m', 0.002).value
        self.sigma_r = p('sigma_rot_rad', 0.002).value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.mount = None
        self.ref = None
        self.ref_t = 0.0
        self.n_ok = self.n_bad = 0
        self.pub = self.create_publisher(Odometry, p('output_topic', '/scan_odom').value, 10)
        self.create_subscription(LaserScan, p('scan_topic', '/scan').value, self._on_scan, qos_profile_sensor_data)

    def _mount(self, frame):
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, frame, rclpy.time.Time()).transform
        except Exception:  # noqa: BLE001
            return None
        return t.translation.x, t.translation.y, _yaw(t.rotation)

    def _on_scan(self, msg):
        if self.mount is None:
            self.mount = self._mount(msg.header.frame_id)
            if self.mount is None:
                return
        pts = transform(scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max),
                        *self.mount)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.ref is None:
            self.ref, self.ref_t = pts, t
            return
        dt = t - self.ref_t
        if dt <= 1e-3:
            return
        res = match(self.ref, pts, max_dist=self.max_dist)
        if (not res.converged or res.rms > self.max_rms or res.inlier_frac < self.min_inlier
                or math.hypot(res.x, res.y) > self.max_step):
            self.n_bad += 1       # keep the old reference, the next scan is matched against it
            if self.n_bad % 20 == 1:
                self.get_logger().warn(f'scan match rejected (rms {res.rms:.3f} m, inliers {res.inlier_frac:.2f}, '
                                       f'converged {res.converged}); {self.n_bad} so far')
            return
        self.n_ok += 1
        self.ref, self.ref_t = pts, t
        vx, vy, wz = body_twist(res, dt)
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.orientation.w = 1.0
        for i in (0, 7, 14, 21, 28, 35):
            out.pose.covariance[i] = 1e6
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.angular.z = vx, vy, wz
        var_v, var_w = (self.sigma_t / dt) ** 2, (self.sigma_r / dt) ** 2
        for i, v in ((0, var_v), (7, var_v), (35, var_w)):
            out.twist.covariance[i] = v
        for i in (14, 21, 28):
            out.twist.covariance[i] = 1e-9
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ScanMatch()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        print(f'scan_match: {node.n_ok} matches used, {node.n_bad} rejected', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
