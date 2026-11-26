import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import PointCloud2, PointField  # noqa: E402

from sim_realism.depth_realism_node import DepthRealism  # noqa: E402
from sim_realism.odom_realism_node import OdomRealism  # noqa: E402


def make_node(cls, **params):
    ctx = rclpy.Context()
    ctx.init(args=['--ros-args'] + [x for k, v in params.items() for x in ('-p', f'{k}:={v}')])
    node = cls.__new__(cls)
    orig = rclpy.node.Node.__init__
    rclpy.node.Node.__init__ = lambda self, name, **kw: orig(self, name, context=ctx, **kw)
    try:
        cls.__init__(node)
    finally:
        rclpy.node.Node.__init__ = orig
    node.pub = Sink()
    node.tf = None
    return node, ctx


class Sink:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


def make_odom(i, x):
    m = Odometry()
    m.header.stamp.sec, m.header.stamp.nanosec = 0, i * 20_000_000
    m.header.frame_id, m.child_frame_id = 'odom', 'base_footprint'
    m.pose.pose.position.x = x
    m.pose.pose.orientation.w = 1.0
    m.twist.twist.linear.x = 0.3
    return m


def test_odom_off_forwards_the_truth_message_unchanged():
    node, ctx = make_node(OdomRealism)
    msg = make_odom(1, 0.123456789)
    node._on_truth(msg)
    assert node.pub.msgs == [msg] and node.pub.msgs[0] is msg
    node.destroy_node()
    ctx.shutdown()


def test_odom_typical_has_latency_covariance_and_keeps_frames():
    node, ctx = make_node(OdomRealism, preset='typical', seed=3, dropout=0.0)
    for i in range(1, 201):
        node._on_truth(make_odom(i, 0.3 * 0.02 * i))
    out = node.pub.msgs
    assert len(out) == 199                         # one period of latency, no dropout
    last = out[-1]
    assert last.header.frame_id == 'odom' and last.child_frame_id == 'base_footprint'
    assert last.pose.covariance[0] > 0 and last.pose.covariance[35] > 0
    assert last.twist.covariance[0] > 0
    assert last.pose.pose.position.x != pytest.approx(0.3 * 0.02 * 200, abs=1e-7)
    assert 'drift' in node.summary()
    node.destroy_node()
    ctx.shutdown()


def make_cloud():
    h, w = 24, 32
    v, u = np.mgrid[0:h, 0:w].astype(np.float32)
    xyz = np.stack([np.full((h, w), 2.0, np.float32), -(u - w / 2) / 40.0 * 2.0, -(v - h / 2) / 40.0 * 2.0], axis=-1)
    xyz[5, 5] = np.nan
    rec = np.zeros((h, w), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('pad', '<f4'), ('rgb', '<u4')])
    rec['x'], rec['y'], rec['z'] = xyz[..., 0], xyz[..., 1], xyz[..., 2]
    rec['rgb'] = 0x00ff8040
    m = PointCloud2()
    m.header.frame_id = 'depth_sensor_link'
    m.header.stamp.sec, m.header.stamp.nanosec = 7, 123
    m.height, m.width = h, w
    m.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1) for n, o in (('x', 0), ('y', 4), ('z', 8))]
    m.fields.append(PointField(name='rgb', offset=16, datatype=PointField.UINT32, count=1))
    m.point_step, m.row_step, m.is_dense, m.is_bigendian = 20, 20 * w, False, False
    m.data = rec.tobytes()
    return m


def depth_node(preset):
    return make_node(DepthRealism, preset=preset, seed=5)


def test_depth_off_forwards_the_message_unchanged():
    node, ctx = depth_node('off')
    msg = make_cloud()
    node._on_cloud(msg)
    assert node.pub.msgs[0] is msg
    node.destroy_node()
    ctx.shutdown()


def test_depth_typical_keeps_organisation_frame_stamp_and_other_fields():
    node, ctx = depth_node('typical')
    msg = make_cloud()
    node._on_cloud(msg)
    out = node.pub.msgs[0]
    assert (out.height, out.width, out.point_step, out.row_step) == (24, 32, 20, 640)
    assert out.header.frame_id == 'depth_sensor_link' and out.header.stamp == msg.header.stamp
    assert [f.name for f in out.fields] == ['x', 'y', 'z', 'rgb']
    assert len(out.data) == len(msg.data)
    a = np.frombuffer(msg.data, np.uint8).reshape(24 * 32, 20)
    b = np.frombuffer(out.data, np.uint8).reshape(24 * 32, 20)
    assert (a[:, 12:] == b[:, 12:]).all()          # padding and rgb untouched
    xyz = np.frombuffer(out.data, '<f4').reshape(24, 32, 5)[..., :3]
    assert np.isnan(xyz[5, 5]).all()
    assert np.isfinite(xyz[..., 0]).mean() > 0.9
    node.destroy_node()
    ctx.shutdown()
