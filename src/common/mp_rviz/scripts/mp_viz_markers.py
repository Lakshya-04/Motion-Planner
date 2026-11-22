#!/usr/bin/env python3
"""Markers no module publishes itself: the mission readout and the tool trace coloured by force.

Reads:   force_topic (geometry_msgs/WrenchStamped; default /ft_conditioner/wrench_conditioned, the topic the
         manipulation manager reads for its force peak),
         feedback_topic (ExecuteCoverage action feedback of the manager: /execute_coverage/_action/feedback;
         the manager has no plain state topic, state and stand are in this feedback),
         TF fixed_frame -> tool_frame (default tcp_link, the frame the manager looks up).
Writes:  /mp_viz/mission_text (visualization_msgs/Marker, TEXT_VIEW_FACING above the tool),
         /mp_viz/tool_trace (visualization_msgs/MarkerArray, one LINE_STRIP of the tool positions, green to red by
         force against force_limit_n, at most trace_max_points points).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402
import rclpy.time  # noqa: E402
from geometry_msgs.msg import Point, WrenchStamped  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from std_msgs.msg import ColorRGBA  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

import mp_viz_logic as logic  # noqa: E402

try:
    from coverage_task_msgs.action import ExecuteCoverage
except ImportError:  # the package is optional: without it the readout shows no state
    ExecuteCoverage = None


class MpVizMarkers(Node):
    def __init__(self):
        super().__init__('mp_viz_markers')
        p = self.declare_parameter
        self.force_topic = p('force_topic', '/ft_conditioner/wrench_conditioned').value
        self.feedback_topic = p('feedback_topic', '/execute_coverage/_action/feedback').value
        self.tool_frame = p('tool_frame', 'tcp_link').value
        self.fixed_frame = p('fixed_frame', 'base_footprint').value
        self.limit = float(p('force_limit_n', 50.0).value)
        max_points = int(p('trace_max_points', 2000).value)
        rate = float(p('rate_hz', 20.0).value)
        self.trace = logic.TraceBuffer(max_points, float(p('trace_min_step_m', 0.002).value))
        self.force = None
        self.fb = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(WrenchStamped, self.force_topic, self._on_force, qos_profile_sensor_data)
        if ExecuteCoverage is not None:
            self.create_subscription(ExecuteCoverage.Impl.FeedbackMessage, self.feedback_topic, self._on_fb, 10)
        else:
            self.get_logger().warn('coverage_task_msgs not found: no mission state in the readout')
        self.text_pub = self.create_publisher(Marker, '/mp_viz/mission_text', 1)
        self.trace_pub = self.create_publisher(MarkerArray, '/mp_viz/tool_trace', 1)
        self.create_timer(1.0 / rate, self._tick)

    def _on_force(self, msg):
        f = msg.wrench.force
        self.force = logic.force_magnitude(f.x, f.y, f.z)

    def _on_fb(self, msg):
        self.fb = msg.feedback

    def _tick(self):
        try:
            t = self.tf_buffer.lookup_transform(self.fixed_frame, self.tool_frame, rclpy.time.Time(),
                                                timeout=Duration(seconds=0.0))
        except Exception:  # no TF yet: nothing to draw
            return
        pos = t.transform.translation
        now = self.get_clock().now().to_msg()
        self.trace.add(pos.x, pos.y, pos.z, self.force if self.force is not None else 0.0)

        fb = self.fb
        text = Marker()
        text.header.frame_id, text.header.stamp = self.fixed_frame, now
        text.ns, text.id, text.type, text.action = 'mp_viz_text', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose.position.x, text.pose.position.y, text.pose.position.z = pos.x, pos.y, pos.z + 0.25
        text.pose.orientation.w = 1.0
        text.scale.z = 0.07
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text.text = logic.format_readout(
            fb.state if fb else '', fb.stand_index if fb else 0, fb.stand_count if fb else 0,
            self.force, self.limit, fb.column if fb else None)
        self.text_pub.publish(text)

        line = Marker()
        line.header.frame_id, line.header.stamp = self.fixed_frame, now
        line.ns, line.id, line.type, line.action = 'mp_viz_trace', 0, Marker.LINE_STRIP, Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.006
        for x, y, z, f in self.trace.points():
            line.points.append(Point(x=x, y=y, z=z))
            r, g, b = logic.force_to_rgb(f, self.limit)
            line.colors.append(ColorRGBA(r=r, g=g, b=b, a=1.0))
        self.trace_pub.publish(MarkerArray(markers=[line]))


def main():
    rclpy.init()
    node = MpVizMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
