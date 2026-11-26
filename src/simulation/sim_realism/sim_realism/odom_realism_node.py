"""Wheel-odometry noise for the simulators: /odom_truth (nav_msgs/Odometry) -> /odom, and TF odom -> base.

Parameters: `preset` (off, good, typical, poor), `seed` (-1 = random, logged), `publish_tf`, `truth_topic`,
`odom_topic`, and one override per model field (see presets.OdomParams; NaN or -1 keeps the preset value).
`/odom_truth` is only read. `~/drift` (std_msgs/Float64MultiArray: position error m, yaw error rad, truth path m)
goes out at 1 Hz and the drift is logged again at shutdown. With preset `off` the truth message is forwarded as it is.
"""
import math
from dataclasses import fields

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

from .odom_model import MessageChannel, OdomModel, quat_to_yaw
from .presets import OdomParams, resolve


class OdomRealism(Node):
    def __init__(self):
        super().__init__('odom_realism')
        # `-p preset:=off` on the command line arrives as a YAML bool
        self.preset = self.declare_parameter('preset', 'off', ParameterDescriptor(dynamic_typing=True)).value
        self.preset = 'off' if self.preset is False else str(self.preset)
        seed = self.declare_parameter('seed', -1).value
        self.publish_tf = self.declare_parameter('publish_tf', True).value
        truth_topic = self.declare_parameter('truth_topic', '/odom_truth').value
        odom_topic = self.declare_parameter('odom_topic', '/odom').value
        overrides = {}
        for f in fields(OdomParams):
            default = -1 if f.name == 'latency_periods' else float('nan')
            value = self.declare_parameter(f.name, default).value
            overrides[f.name] = None if (f.name == 'latency_periods' and value < 0) else value
        self.params = resolve('odom', self.preset, overrides)
        self.seed = int(seed) if seed >= 0 else int(np.random.SeedSequence().entropy % (2 ** 31))
        self.model = OdomModel(self.params, self.seed)
        self.channel = MessageChannel(self.params.latency_periods, self.params.dropout, self.seed + 1)
        self.passthrough = self.params.is_identity()
        self.n_in = self.n_out = 0
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        self.pub = self.create_publisher(Odometry, odom_topic, 10)
        self.drift_pub = self.create_publisher(Float64MultiArray, '~/drift', 10)
        self.create_subscription(Odometry, truth_topic, self._on_truth, 10)
        self.create_timer(1.0, self._publish_drift)
        self.get_logger().info(
            f'odom_realism: preset={self.preset} seed={self.seed} {truth_topic} -> {odom_topic}, tf={self.publish_tf}; '
            f'scale error this run: linear {self.model.lin_err * 100:+.2f} %, yaw {self.model.yaw_err * 100:+.2f} %, '
            f'initial yaw bias {math.degrees(self.model.bias):+.3f} deg/s, latency {self.params.latency_periods} periods')

    def _on_truth(self, msg):
        self.n_in += 1
        if self.passthrough:
            self._emit(msg)
            return
        po, tw = msg.pose.pose, msg.twist.twist
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x, y, yaw, (vx, vy, wz), pcov, tcov = self.model.step(
            po.position.x, po.position.y, quat_to_yaw(*(getattr(po.orientation, k) for k in 'xyzw')), t,
            (tw.linear.x, tw.linear.y, tw.angular.z))
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = x, y, po.position.z
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        out.pose.covariance = pcov.tolist()
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.angular.z = vx, vy, wz
        out.twist.covariance = tcov.tolist()
        released = self.channel.push(out)
        if released is not None:
            self._emit(released)

    def _emit(self, msg):
        self.n_out += 1
        self.pub.publish(msg)
        if self.tf is not None:
            t = TransformStamped()
            t.header = msg.header
            t.child_frame_id = msg.child_frame_id
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation
            self.tf.sendTransform(t)

    def _publish_drift(self):
        pos, yaw, path = self.model.drift()
        self.drift_pub.publish(Float64MultiArray(data=[pos, yaw, path]))

    def summary(self):
        pos, yaw, path = self.model.drift()
        pct = 100.0 * pos / path if path > 0.05 else float('nan')
        return (f'odom_realism drift: preset={self.preset} seed={self.seed} position error {pos:.3f} m '
                f'({pct:.2f} % of {path:.2f} m driven), yaw error {math.degrees(yaw):+.2f} deg, '
                f'slips {self.model.slips}, messages in {self.n_in} out {self.n_out}')


def main():
    rclpy.init()
    node = OdomRealism()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        print(node.summary(), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
