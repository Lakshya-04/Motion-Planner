#!/usr/bin/env python3
"""
Example script demonstrating how to use the Coverage Planner service.

This script shows how to:
1. Connect to the coverage planner service
2. Request a coverage path with custom parameters
3. Process and display the results
"""

import rclpy
from rclpy.node import Node
from coverage_planner.srv import GenerateCoveragePath
import time


class CoveragePlannerExample(Node):
    def __init__(self):
        super().__init__("coverage_planner_example")

        self.get_logger().info("Coverage Planner Example Client")

        # Create service client
        self.client = self.create_client(
            GenerateCoveragePath, "/coverage_planner_node/generate_coverage_path"
        )

        # Wait for service to be available
        self.get_logger().info("Waiting for coverage planner service...")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        self.get_logger().info("Service is ready!")

    def generate_coverage_path(
        self,
        pattern="boustrophedon",
        swath=0.5,
        step=0.4,
        direction="horizontal",
        corner="top_left",
    ):
        """
        Request a coverage path from the service.

        Args:
            pattern: Coverage pattern name
            swath: Effective coverage width (meters)
            step: Distance between sweep lines (meters)
            direction: 'horizontal' or 'vertical'
            corner: Start corner ('top_left', 'top_right', etc.)
        """
        self.get_logger().info(f"Requesting {pattern} coverage path...")
        self.get_logger().info(f"  Swath: {swath}m, Step: {step}m")
        self.get_logger().info(f"  Direction: {direction}, Start: {corner}")

        # Create request
        request = GenerateCoveragePath.Request()
        request.pattern = pattern
        request.swath = swath
        request.step = step
        request.sweep_direction = direction
        request.start_corner = corner

        # Call service
        future = self.client.call_async(request)

        self.get_logger().info("Waiting for response...")
        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        # Process response
        if response.success:
            self.get_logger().info("✓ Coverage path generated successfully!")
            self.get_logger().info("─" * 50)
            self.get_logger().info(f"Pattern Used:    {response.pattern_used}")
            self.get_logger().info(f"Waypoints:       {response.num_waypoints}")
            self.get_logger().info(f"Total Distance:  {response.total_distance:.2f} m")
            self.get_logger().info(f"Coverage Area:   {response.coverage_area:.2f} m²")
            self.get_logger().info(f"Planning Time:   {response.planning_time:.3f} s")
            self.get_logger().info("─" * 50)

            # Path frame and timestamp
            if response.path.poses:
                self.get_logger().info(
                    f"Path Frame:      {response.path.header.frame_id}"
                )
                self.get_logger().info(
                    f"First Waypoint:  ({response.path.poses[0].pose.position.x:.2f}, "
                    f"{response.path.poses[0].pose.position.y:.2f})"
                )
                self.get_logger().info(
                    f"Last Waypoint:   ({response.path.poses[-1].pose.position.x:.2f}, "
                    f"{response.path.poses[-1].pose.position.y:.2f})"
                )
        else:
            self.get_logger().error("✗ Failed to generate coverage path")
            self.get_logger().error(f"Reason: {response.message}")

        return response


def main(args=None):
    rclpy.init(args=args)

    example = CoveragePlannerExample()

    try:
        # Example 1: Default boustrophedon pattern
        example.get_logger().info("\n" + "=" * 50)
        example.get_logger().info("EXAMPLE 1: Default Boustrophedon")
        example.get_logger().info("=" * 50)
        example.generate_coverage_path()

        time.sleep(2)

        # Example 2: Vertical sweeps with larger swath
        example.get_logger().info("\n" + "=" * 50)
        example.get_logger().info("EXAMPLE 2: Vertical Sweeps")
        example.get_logger().info("=" * 50)
        example.generate_coverage_path(
            swath=0.8, step=0.6, direction="vertical", corner="bottom_left"
        )

        time.sleep(2)

        # Example 3: Horizontal with tight spacing
        example.get_logger().info("\n" + "=" * 50)
        example.get_logger().info("EXAMPLE 3: Tight Spacing (more overlap)")
        example.get_logger().info("=" * 50)
        example.generate_coverage_path(
            swath=0.5,
            step=0.3,  # 40% overlap
            direction="horizontal",
            corner="top_right",
        )

        example.get_logger().info("\n" + "=" * 50)
        example.get_logger().info("All examples completed!")
        example.get_logger().info("=" * 50)

    except KeyboardInterrupt:
        example.get_logger().info("Interrupted by user")
    finally:
        example.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
