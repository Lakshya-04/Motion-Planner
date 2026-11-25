#!/usr/bin/env python3
"""Kinematic stand-in for the mobile base: integrates /cmd_vel into /odom and the TF odom -> base_footprint.

Perfect tracking, no wheels, no physics: the commanded body twist (geometry_msgs/Twist in base_footprint, what
stand_navigator publishes) is integrated exactly, so the base 'drives' to the stand the way the plan says. Used by
scripts/plan_playback.sh, where the arm runs on ros2_control mock hardware and there is no simulator.

Parameters: odom_frame (odom), base_frame (base_footprint), cmd_vel_topic (cmd_vel), odom_topic (odom),
rate_hz (50), cmd_timeout_s (0.5; no command for that long = the base stops), x/y/yaw (start pose, 0).
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist, TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402

from mp_fake_base_logic import integrate  # noqa: E402


class FakeBase(Node):
    def __init__(self):
        super().__init__('mp_fake_base')
        p = self.declare_parameter
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.timeout = float(p('cmd_timeout_s', 0.5).value)
        rate = float(p('rate_hz', 50.0).value)
        self.x, self.y, self.yaw = float(p('x', 0.0).value), float(p('y', 0.0).value), float(p('yaw', 0.0).value)
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_t = self.get_clock().now()
        self.last = self.get_clock().now()
        self.create_subscription(Twist, p('cmd_vel_topic', 'cmd_vel').value, self._on_cmd, 10)
        self.pub = self.create_publisher(Odometry, p('odom_topic', 'odom').value, 10)
        self.tf = TransformBroadcaster(self)
        self.create_timer(1.0 / rate, self._tick)

    def _on_cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)
        self.cmd_t = self.get_clock().now()

    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self.last).nanoseconds * 1e-9
        self.last = now
        if (now - self.cmd_t).nanoseconds * 1e-9 > self.timeout:
            self.cmd = (0.0, 0.0, 0.0)
        vx, vy, wz = self.cmd
        self.x, self.y, self.yaw = integrate(self.x, self.y, self.yaw, vx, vy, wz, dt)
        qz, qw = math.sin(0.5 * self.yaw), math.cos(0.5 * self.yaw)
        stamp = now.to_msg()
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, self.odom_frame, self.base_frame
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf.sendTransform(t)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, self.odom_frame, self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = vx, vy, wz
        self.pub.publish(o)


def main():
    rclpy.init()
    rclpy.spin(FakeBase())


if __name__ == '__main__':
    main()
