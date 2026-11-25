#!/usr/bin/env python3
"""Writes the marker topics as sim-neutral primitives to a JSON file, atomically, for the MuJoCo viewer process.

Parameters: topics (default /mp_viz/plan_columns, /mp_viz/stands), path (default /tmp/mp_sim_overlay.json),
frame_x/frame_y/frame_z/frame_yaw (marker frame -> sim world, default identity).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402

from _common import declare_common, subscribe  # noqa: E402
from sim_overlay.mujoco_overlay import save_overlay  # noqa: E402
from sim_overlay.overlay import FrameTransform, OverlayState  # noqa: E402


def main():
    rclpy.init()
    node = Node('overlay_dump')
    c = declare_common(node)
    path = node.declare_parameter('path', '/tmp/mp_sim_overlay.json').value
    state = OverlayState(FrameTransform(*c['tf']))

    def write():
        prims = state.primitives()
        save_overlay(path, prims)
        node.get_logger().info('wrote %d primitive(s) to %s' % (len(prims), path))
    subscribe(node, c['topics'], state, write)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
