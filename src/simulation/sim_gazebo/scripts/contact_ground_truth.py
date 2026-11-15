#!/usr/bin/env python3
"""Ground-truth contact wrench on the tool, for evaluation only.

Sums the contact forces gz reports on the tool collision (contact sensor `tool_contact`) and
expresses them at tcp_link, in the tcp_link axes: the wrench the ENVIRONMENT exerts on the tool.
Convention matches the target of a wrench conditioner: pressing along tool +z gives force.z < 0.

  in : /tool_contact/contacts   (ros_gz_interfaces/Contacts, bridged from gz)
  out: /ft_sensor/ground_truth_wrench   (geometry_msgs/WrenchStamped, frame tcp_link)
Never feed this to a controller: a real robot has no such signal.
"""
import numpy as np
import rclpy
import rclpy.duration
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.time import Time
from ros_gz_interfaces.msg import Contacts
from tf2_ros import Buffer, TransformListener

TOOL_BODY = 'tool_tip_link'   # the tool's collision (spring-loaded tip, urdf/gazebo_tags.xacro)


def _quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tool_wrench_world(contacts, tcp_pos):
    """(force, torque about tcp_pos), world axes, from a Contacts message."""
    force, torque = np.zeros(3), np.zeros(3)
    for c in contacts.contacts:
        for pos, w in zip(c.positions, c.wrenches):
            # gz leaves the body names of the wrench empty: the collision entities say which side is the tool
            if TOOL_BODY in c.collision1.name:
                wr = w.body_1_wrench
            elif TOOL_BODY in c.collision2.name:
                wr = w.body_2_wrench
            else:
                continue
            f = np.array([wr.force.x, wr.force.y, wr.force.z])
            t = np.array([wr.torque.x, wr.torque.y, wr.torque.z])
            p = np.array([pos.x, pos.y, pos.z])
            force += f
            torque += t + np.cross(p - tcp_pos, f)
    return force, torque


class GroundTruth(Node):
    def __init__(self):
        super().__init__('contact_ground_truth')
        self.declare_parameter('tcp_frame', 'tcp_link')
        self.declare_parameter('world_frame', 'world')
        self.tcp = self.get_parameter('tcp_frame').value
        self.world = self.get_parameter('world_frame').value
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(WrenchStamped, '/ft_sensor/ground_truth_wrench', 10)
        self.last = self.get_clock().now()
        self.create_subscription(Contacts, '/tool_contact/contacts', self.on_contacts, 10)
        self.create_timer(0.05, self.publish_zero)

    def publish_zero(self):
        # the contact sensor only publishes while something touches the tool
        if self.get_clock().now() - self.last > rclpy.duration.Duration(seconds=0.1):
            out = WrenchStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.tcp
            self.pub.publish(out)

    def on_contacts(self, msg):
        self.last = self.get_clock().now()
        try:
            t = self.tf.lookup_transform(self.world, self.tcp, Time())
        except Exception:  # noqa: BLE001  (tf not up yet)
            return
        R = _quat_matrix(t.transform.rotation)
        p = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        f, tau = tool_wrench_world(msg, p)
        out = WrenchStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.tcp
        out.wrench.force.x, out.wrench.force.y, out.wrench.force.z = (R.T @ f).tolist()
        out.wrench.torque.x, out.wrench.torque.y, out.wrench.torque.z = (R.T @ tau).tolist()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = GroundTruth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
