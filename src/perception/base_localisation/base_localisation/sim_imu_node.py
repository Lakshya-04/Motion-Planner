"""Derived IMU for the MuJoCo sim: nav_msgs/Odometry (truth) -> sensor_msgs/Imu.

Gazebo has a real `imu` sensor; this is for MuJoCo, whose base is a position-held planar joint. The orientation
is not provided (covariance[0] = -1); angular velocity and linear acceleration come from the twist of the
odometry it listens to. Parameters: `odom_topic`, `imu_topic`, `frame_id`, `gyro_noise`, `gyro_bias`,
`accel_noise`, `accel_bias` (rad/s, rad/s, m/s^2, m/s^2; all 0 = ideal), `seed` (-1 = random, logged).
"""
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

from .sim_imu import ImuModel


class SimImu(Node):
    def __init__(self):
        super().__init__('sim_imu')
        p = self.declare_parameter
        seed = p('seed', -1).value
        self.seed = int(seed) if seed >= 0 else int(np.random.SeedSequence().entropy % (2 ** 31))
        self.frame = p('frame_id', 'imu_link').value
        self.model = ImuModel(p('gyro_noise', 0.0).value, p('gyro_bias', 0.0).value, p('accel_noise', 0.0).value,
                              p('accel_bias', 0.0).value, self.seed)
        self.pub = self.create_publisher(Imu, p('imu_topic', '/imu').value, 10)
        self.create_subscription(Odometry, p('odom_topic', '/odom').value, self._on_odom, 10)
        self.get_logger().info(f'sim_imu: seed={self.seed} gyro bias {self.model.gyro_bias} rad/s, '
                               f'accel bias {self.model.accel_bias} m/s^2')

    def _on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        tw = msg.twist.twist
        gyro, accel = self.model.step(t, tw.linear.x, tw.linear.y, tw.angular.z)
        out = Imu()
        out.header.stamp, out.header.frame_id = msg.header.stamp, self.frame
        out.orientation_covariance[0] = -1.0
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = map(float, gyro)
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = map(float, accel)
        gv, av = self.model.covariances()
        for i in (0, 4, 8):
            out.angular_velocity_covariance[i] = gv
            out.linear_acceleration_covariance[i] = av
        self.pub.publish(out)


def main():
    rclpy.init()
    try:
        rclpy.spin(SimImu())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
