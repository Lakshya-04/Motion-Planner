"""Depth noise for the simulators: organised PointCloud2 on `cloud_in` -> `cloud_out`, same fields, size, frame and stamp.

Parameters: `preset` (off, good, typical, poor), `seed` (-1 = random, logged), `cloud_in`
(/depth_sensor/points_truth), `cloud_out` (/depth_sensor/points), and one override per model field (see
presets.DepthParams; NaN keeps the preset value). With preset `off` the input message is forwarded as it is.
"""
from dataclasses import fields

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from .depth_model import apply_depth_model
from .presets import DepthParams, resolve


def xyz_views(msg, data):
    """Writable float32 (H, W) views of x, y, z inside a bytearray of the cloud's data."""
    off = {f.name: f.offset for f in msg.fields if f.name in 'xyz' and f.datatype == 7}
    if len(off) != 3 or msg.is_bigendian:
        raise ValueError('cloud needs little-endian float32 x, y, z fields')
    h, w = msg.height, msg.width
    strides = (msg.row_step if h > 1 else msg.point_step * w, msg.point_step)
    return [np.ndarray((h, w), np.float32, data, offset=off[k], strides=strides) for k in 'xyz']


class DepthRealism(Node):
    def __init__(self):
        super().__init__('depth_realism')
        # `-p preset:=off` on the command line arrives as a YAML bool
        self.preset = self.declare_parameter('preset', 'off', ParameterDescriptor(dynamic_typing=True)).value
        self.preset = 'off' if self.preset is False else str(self.preset)
        seed = self.declare_parameter('seed', -1).value
        cloud_in = self.declare_parameter('cloud_in', '/depth_sensor/points_truth').value
        cloud_out = self.declare_parameter('cloud_out', '/depth_sensor/points').value
        overrides = {f.name: self.declare_parameter(f.name, float('nan')).value for f in fields(DepthParams)}
        self.params = resolve('depth', self.preset, overrides)
        self.seed = int(seed) if seed >= 0 else int(np.random.SeedSequence().entropy % (2 ** 31))
        self.rng = np.random.default_rng(self.seed)
        self.n = 0
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(PointCloud2, cloud_out, qos)
        self.create_subscription(PointCloud2, cloud_in, self._on_cloud, qos)
        self.get_logger().info(f'depth_realism: preset={self.preset} seed={self.seed} {cloud_in} -> {cloud_out}: {self.params}')

    def _on_cloud(self, msg):
        if self.params.is_identity():
            self.pub.publish(msg)
            return
        out = PointCloud2()
        out.header, out.height, out.width = msg.header, msg.height, msg.width
        out.fields, out.is_bigendian, out.point_step = msg.fields, msg.is_bigendian, msg.point_step
        out.row_step, out.is_dense = msg.row_step, False
        data = bytearray(msg.data)
        try:
            views = xyz_views(msg, data)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=10.0)
            return
        xyz = np.stack(views, axis=-1)
        new, stats = apply_depth_model(xyz, self.params, self.rng)
        for i, v in enumerate(views):
            v[...] = new[..., i]
        out.data = bytes(data)
        self.pub.publish(out)
        self.n += 1
        if self.n % 25 == 1:
            self.get_logger().info(f'depth_realism cloud {self.n}: {stats}')


def main():
    rclpy.init()
    node = DepthRealism()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
