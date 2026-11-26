#!/usr/bin/env python3
"""Ground-truth contact wrench on the tool, for evaluation only.

Sums the contact forces gz reports on the tool collision (contact sensor `tool_contact`) and
expresses them at tcp_link, in the tcp_link axes: the wrench the ENVIRONMENT exerts on the tool.
Convention matches the target of a wrench conditioner: pressing along tool +z gives force.z < 0.

  in : /tool_contact/contacts   (ros_gz_interfaces/Contacts, bridged from gz)
  out: /ft_sensor/ground_truth_wrench   (geometry_msgs/WrenchStamped, frame tcp_link)
       /ft_sensor/ground_truth_info     (std_msgs/Float64MultiArray, one per Contacts message:
            [pairs, points, filtered_pairs, sum_point_norm_N, max_point_norm_N, duplicate_points];
            sum_point_norm >> |force| means points cancel or are counted twice, see docs/sim_gazebo.md)
       /ft_sensor/ground_truth_others   (std_msgs/String, one per Contacts message: the names of the other
            collisions the tool touches, comma separated; says WHAT was hit when a spike shows up)
Parameters include_other / exclude_other (lists of substrings of the other collision's name, default empty =
every pair): e.g. include_other:=['wall'] sums only tool-wall pairs. Behaviour is unchanged with the defaults.
Never feed this to a controller: a real robot has no such signal.
"""
import numpy as np
import rclpy
import rclpy.duration
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.time import Time
from ros_gz_interfaces.msg import Contacts
from sim_gazebo.contact_summary import tool_wrench_world
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import Buffer, TransformListener

TOOL_BODY = 'tool_tip_link'   # the tool's collision (spring-loaded tip, urdf/gazebo_tags.xacro)


def _quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class GroundTruth(Node):
    def __init__(self):
        super().__init__('contact_ground_truth')
        self.declare_parameter('tcp_frame', 'tcp_link')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('include_other', [''])
        self.declare_parameter('exclude_other', [''])
        self.include = [x for x in self.get_parameter('include_other').value if x]
        self.exclude = [x for x in self.get_parameter('exclude_other').value if x]
        self.tcp = self.get_parameter('tcp_frame').value
        self.world = self.get_parameter('world_frame').value
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(WrenchStamped, '/ft_sensor/ground_truth_wrench', 10)
        self.info_pub = self.create_publisher(Float64MultiArray, '/ft_sensor/ground_truth_info', 10)
        self.others_pub = self.create_publisher(String, '/ft_sensor/ground_truth_others', 10)
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
        f, tau, st = tool_wrench_world(msg, p, TOOL_BODY, self.include, self.exclude)
        info = Float64MultiArray()
        info.data = [float(st['contacts']), float(st['points']), float(st['filtered']),
                     st['sum_point_norm'], st['max_point_norm'], float(st['dup_points'])]
        self.info_pub.publish(info)
        self.others_pub.publish(String(data=','.join(st['others'])))
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
