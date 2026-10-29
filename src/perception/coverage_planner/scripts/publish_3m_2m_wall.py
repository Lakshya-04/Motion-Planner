#!/usr/bin/python3
"""
Node to publish a 3m x 2m vertical wall occupancy grid and trigger the Boustrophedon coverage planner.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from coverage_planner.srv import GenerateCoveragePath
import numpy as np


class Publish3m2mWall(Node):
    def __init__(self):
        super().__init__("publish_3m_2m_wall")

        self.declare_parameter("width_m", 3.0)
        self.declare_parameter("height_m", 3.0)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("offset_x", 0.70)
        self.declare_parameter("offset_y", -0.25)
        self.declare_parameter("offset_z", 0.40)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("sweep_direction", "vertical")
        self.declare_parameter("start_corner", "top_left")
        self.declare_parameter("obstacle_profile", "none")
        self.declare_parameter("dynamic_obstacles", False)
        self.declare_parameter("request_path_on_start", True)

        self.width_m = self.get_parameter("width_m").value
        self.height_m = self.get_parameter("height_m").value
        self.resolution = self.get_parameter("resolution").value
        self.offset_x = self.get_parameter("offset_x").value
        self.offset_y = self.get_parameter("offset_y").value
        self.offset_z = self.get_parameter("offset_z").value
        self.frame_id = self.get_parameter("frame_id").value
        self.sweep_direction = self.get_parameter("sweep_direction").value
        self.start_corner = self.get_parameter("start_corner").value
        self.obstacle_profile = self.get_parameter("obstacle_profile").value
        self.dynamic_obstacles = self.get_parameter("dynamic_obstacles").value
        self.request_path_on_start = self.get_parameter("request_path_on_start").value
        self.tick = 0

        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 1)
        self.trial_map_pub = self.create_publisher(OccupancyGrid, "/trial_map", 1)

        # Create client for coverage planner service
        self.planner_client = self.create_client(
            GenerateCoveragePath, "/coverage_planner_node/generate_coverage_path"
        )

        # Periodic timer to republish the map
        self.timer = self.create_timer(1.0, self.publish_map)

        self.grid_msg = self.generate_wall_grid(phase=0)
        self.path_requested = False

        self.get_logger().info(
            "Publish3m2mWall initialized. Ready to publish wall and request coverage path."
        )

    def _fill_rect(self, data, x0, x1, y0, y1):
        x0 = max(0, min(data.shape[1], x0))
        x1 = max(0, min(data.shape[1], x1))
        y0 = max(0, min(data.shape[0], y0))
        y1 = max(0, min(data.shape[0], y1))
        if x1 > x0 and y1 > y0:
            data[y0:y1, x0:x1] = 100

    def _apply_obstacles(self, data, phase):
        profile = str(self.obstacle_profile).strip().lower()
        h, w = data.shape

        if profile == "none":
            return

        if profile == "center_block":
            bw = max(2, int(0.18 * w))
            bh = max(2, int(0.22 * h))
            cx = w // 2
            cy = h // 2
            self._fill_rect(
                data, cx - bw // 2, cx + bw // 2, cy - bh // 2, cy + bh // 2
            )
            return

        if profile == "pillars":
            pillar_w = max(1, int(0.06 * w))
            for ratio in (0.2, 0.5, 0.8):
                col = int(ratio * w)
                self._fill_rect(data, col, col + pillar_w, int(0.1 * h), int(0.9 * h))
            return

        if profile == "window_frame":
            frame_t = max(1, int(0.05 * min(h, w)))
            self._fill_rect(data, 0, w, 0, frame_t)
            self._fill_rect(data, 0, w, h - frame_t, h)
            self._fill_rect(data, 0, frame_t, 0, h)
            self._fill_rect(data, w - frame_t, w, 0, h)
            return

        if profile == "moving_bar":
            bar_h = max(2, int(0.08 * h))
            span = max(1, h - bar_h)
            y0 = phase % span
            self._fill_rect(data, int(0.1 * w), int(0.9 * w), y0, y0 + bar_h)
            return

        self.get_logger().warn(
            f"Unknown obstacle_profile='{self.obstacle_profile}', using none"
        )

    def generate_wall_grid(self, phase=0):
        width_cells = int(round(self.width_m / self.resolution))
        height_cells = int(round(self.height_m / self.resolution))

        self.get_logger().info(
            f"Generating wall: {self.width_m}m x {self.height_m}m "
            f"({width_cells} x {height_cells} cells), "
            f"frame={self.frame_id}, bottom Z = {self.offset_z}m"
        )

        data = np.zeros((height_cells, width_cells), dtype=np.int8)
        self._apply_obstacles(data, phase)

        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.info.width = width_cells
        msg.info.height = height_cells
        msg.info.resolution = self.resolution

        msg.info.origin.position.x = self.offset_x
        msg.info.origin.position.y = self.offset_y
        msg.info.origin.position.z = (
            self.offset_z
        )  # ← controls wall bottom Z in world frame

        # Quaternion (x=0.5, y=0.5, z=0.5, w=0.5):
        # grid local X → world Y  (wall width direction)
        # grid local Y → world Z  (wall height direction, upward)
        # grid local Z → world X  (wall normal direction)
        msg.info.origin.orientation.x = 0.5
        msg.info.origin.orientation.y = 0.5
        msg.info.origin.orientation.z = 0.5
        msg.info.origin.orientation.w = 0.5

        msg.data = data.flatten().tolist()
        return msg

    def publish_map(self):
        if self.dynamic_obstacles:
            self.tick += 1
            self.grid_msg = self.generate_wall_grid(phase=self.tick)

        self.grid_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.grid_msg)
        self.trial_map_pub.publish(self.grid_msg)

        if (
            self.request_path_on_start
            and not self.path_requested
            and self.planner_client.wait_for_service(timeout_sec=1.0)
        ):
            self.request_path()
            self.path_requested = True

    def request_path(self):
        req = GenerateCoveragePath.Request()
        req.map = self.grid_msg
        req.swath = 0.25
        req.step = 0.20  # Swath - Overlap = 0.25 - 0.05
        req.sweep_direction = self.sweep_direction
        req.start_corner = self.start_corner

        future = self.planner_client.call_async(req)
        future.add_done_callback(self.path_callback)
        self.get_logger().info(
            f"Requested coverage path with direction: {self.sweep_direction}"
        )

    def path_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f"Received path with {len(response.path.poses)} poses"
            )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Publish3m2mWall()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
