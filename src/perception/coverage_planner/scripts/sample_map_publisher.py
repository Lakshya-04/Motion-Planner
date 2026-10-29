#!/usr/bin/env python3
"""
ROS 2 node to publish a sample occupancy grid map for coverage planner testing.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np


class SampleMapPublisher(Node):
    def __init__(self):
        super().__init__("sample_map_publisher")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("vertical_wall", False)
        self.declare_parameter("offset_x", 1.0)
        self.declare_parameter("offset_y", -0.5)
        self.declare_parameter("offset_z", 1.0)
        self.declare_parameter("orientation_offset_r", 0.0)  # Roll offset (deg)
        self.declare_parameter("orientation_offset_p", 0.0)  # Pitch offset (deg)
        self.declare_parameter("orientation_offset_y", 0.0)  # Yaw offset (deg)
        self.publisher_ = self.create_publisher(OccupancyGrid, "/trial_map", 1)
        self.timer_ = self.create_timer(1.0, self.publish_map)
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self.vertical_wall = (
            self.get_parameter("vertical_wall").get_parameter_value().bool_value
        )
        self.offset_x = (
            self.get_parameter("offset_x").get_parameter_value().double_value
        )
        self.offset_y = (
            self.get_parameter("offset_y").get_parameter_value().double_value
        )
        self.offset_z = (
            self.get_parameter("offset_z").get_parameter_value().double_value
        )
        self.orientation_offset_r = (
            self.get_parameter("orientation_offset_r")
            .get_parameter_value()
            .double_value
        )
        self.orientation_offset_p = (
            self.get_parameter("orientation_offset_p")
            .get_parameter_value()
            .double_value
        )
        self.orientation_offset_y = (
            self.get_parameter("orientation_offset_y")
            .get_parameter_value()
            .double_value
        )
        self.get_logger().info(
            f"SampleMapPublisher started, publishing to /trial_map with frame_id={self.frame_id}, vertical_wall={self.vertical_wall}, offset=({self.offset_x}, {self.offset_y}, {self.offset_z})"
        )

    def publish_map(self):
        # Map parameters
        width = 5
        height = 5
        resolution = 0.3  # meters/cell
        origin_x = 0.0
        origin_y = 0.0

        # Create map data: 0=free, 100=occupied, -1=unknown
        data = np.zeros((height, width), dtype=np.int8)
        # Add a border
        data[0, :] = 100
        data[-1, :] = 100
        data[:, 0] = 100
        data[:, -1] = 100
        # Add a central obstacle
        data[20:30, 20:30] = 100
        # Add some unknown cells
        data[10:15, 40:45] = -1

        # Flatten for OccupancyGrid
        flat_data = data.flatten().tolist()

        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.info.width = width
        msg.info.height = height
        msg.info.resolution = resolution

        import math

        def euler_to_quaternion(roll, pitch, yaw):
            # Convert degrees to radians
            roll = math.radians(roll)
            pitch = math.radians(pitch)
            yaw = math.radians(yaw)
            qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(
                yaw / 2
            ) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
            qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(
                yaw / 2
            ) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
            qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(
                yaw / 2
            ) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
            qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(
                yaw / 2
            ) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
            return (qx, qy, qz, qw)

        # Compose base orientation (vertical wall or horizontal)
        if self.vertical_wall:
            # For a vertical wall, set the origin and rotate so X=horizontal, Z=vertical
            msg.info.origin.position.x = origin_x + self.offset_x
            msg.info.origin.position.y = self.offset_y
            msg.info.origin.position.z = origin_y + self.offset_z
            # Quaternion for 90deg about Y: (w, x, y, z) = (sqrt(0.5), 0, sqrt(0.5), 0)
            base_q = (0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5))
        else:
            # Standard horizontal map
            msg.info.origin.position.x = origin_x + self.offset_x
            msg.info.origin.position.y = origin_y + self.offset_y
            msg.info.origin.position.z = self.offset_z
            base_q = (0.0, 0.0, 0.0, 1.0)

        # Apply orientation offset (roll, pitch, yaw in degrees)
        offset_q = euler_to_quaternion(
            self.orientation_offset_r,
            self.orientation_offset_p,
            self.orientation_offset_y,
        )

        # Quaternion multiplication: q = base_q * offset_q
        def quat_mult(q1, q2):
            x1, y1, z1, w1 = q1
            x2, y2, z2, w2 = q2
            x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
            y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
            z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
            w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
            return (x, y, z, w)

        qx, qy, qz, qw = quat_mult(base_q, offset_q)
        msg.info.origin.orientation.x = qx
        msg.info.origin.orientation.y = qy
        msg.info.origin.orientation.z = qz
        msg.info.origin.orientation.w = qw

        msg.data = flat_data

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Published sample map with frame_id={self.frame_id}, vertical_wall={self.vertical_wall}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SampleMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
