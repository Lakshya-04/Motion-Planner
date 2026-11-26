#!/usr/bin/env python3
"""Odometry and TF for the planar base: `sim_mujoco/base_pose` (PoseStamped in the odom frame) and
`sim_mujoco/base_twist` (TwistStamped, body frame) from the MuJoCo plugin -> nav_msgs/Odometry on `odom`
and TF `odom -> <base_frame>` (default base_footprint). With `odom_topic:=odom_truth` and `publish_tf:=false`
(launch `odom_realism` on) it only publishes the truth; sim_realism makes /odom and the TF.

A separate node keeps the ros2_control plugin free of nav_msgs / tf2 (it publishes geometry_msgs only).
"""
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class BaseOdom(Node):
    def __init__(self):
        super().__init__('base_odom')
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.twist = TwistStamped()
        self.tf = TransformBroadcaster(self) if self.declare_parameter('publish_tf', True).value else None
        self.pub = self.create_publisher(Odometry, self.declare_parameter('odom_topic', 'odom').value, 10)
        self.create_subscription(TwistStamped, 'sim_mujoco/base_twist', self._twist, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'sim_mujoco/base_pose', self._pose, qos_profile_sensor_data)

    def _twist(self, msg):
        self.twist = msg

    def _pose(self, msg):
        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = self.base_frame
        odom.pose.pose = msg.pose
        odom.twist.twist = self.twist.twist
        self.pub.publish(odom)
        if self.tf is None:
            return
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = msg.pose.position.x
        tf.transform.translation.y = msg.pose.position.y
        tf.transform.translation.z = msg.pose.position.z
        tf.transform.rotation = msg.pose.orientation
        self.tf.sendTransform(tf)


def main():
    rclpy.init()
    rclpy.spin(BaseOdom())


if __name__ == '__main__':
    main()
