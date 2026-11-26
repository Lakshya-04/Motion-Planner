import pytest

rclpy = pytest.importorskip('rclpy')
from nav_msgs.msg import Odometry  # noqa: E402

from base_localisation.sim_imu_node import SimImu  # noqa: E402


class Sink:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


def make_node(**params):
    ctx = rclpy.Context()
    ctx.init(args=['--ros-args'] + [x for k, v in params.items() for x in ('-p', f'{k}:={v}')])
    node = SimImu.__new__(SimImu)
    orig = rclpy.node.Node.__init__
    rclpy.node.Node.__init__ = lambda self, name, **kw: orig(self, name, context=ctx, **kw)
    try:
        SimImu.__init__(node)
    finally:
        rclpy.node.Node.__init__ = orig
    node.pub = Sink()
    return node, ctx


def odom(i, vx, wz):
    m = Odometry()
    m.header.stamp.sec, m.header.stamp.nanosec = 0, i * 10_000_000
    m.twist.twist.linear.x, m.twist.twist.angular.z = vx, wz
    return m


def test_imu_message_from_a_turning_twist():
    node, ctx = make_node()
    for i in range(5):
        node._on_odom(odom(i, 0.2, 0.3))
    out = node.pub.msgs[-1]
    assert out.header.frame_id == 'imu_link'
    assert out.angular_velocity.z == pytest.approx(0.3) and out.linear_acceleration.z == pytest.approx(9.81)
    assert out.orientation_covariance[0] == -1.0
    assert out.angular_velocity_covariance[8] == pytest.approx(1e-6)
    node.destroy_node()


def test_noise_params_reach_the_model_and_the_covariance():
    node, ctx = make_node(gyro_noise=0.01, seed=5)
    node._on_odom(odom(0, 0.0, 0.0))
    node._on_odom(odom(1, 0.0, 0.0))
    out = node.pub.msgs[-1]
    assert out.angular_velocity.z != 0.0
    assert out.angular_velocity_covariance[8] == pytest.approx(1e-4)
    node.destroy_node()
