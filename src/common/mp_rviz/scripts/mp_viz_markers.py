#!/usr/bin/env python3
"""Markers no module publishes itself: the mission readout and the tool trace coloured by force.

Reads:   force_topic (geometry_msgs/WrenchStamped; default /ft_conditioner/wrench_conditioned, the topic the
         manipulation manager reads for its force peak),
         feedback_topic (ExecuteCoverage action feedback of the manager: /execute_coverage/_action/feedback;
         the manager has no plain state topic, state and stand are in this feedback),
         TF fixed_frame -> tool_frame (default tcp_link, the frame the manager looks up).
Writes:  /mp_viz/mission_text (visualization_msgs/Marker, TEXT_VIEW_FACING above the tool),
         /mp_viz/tool_trace (visualization_msgs/MarkerArray, one LINE_STRIP of the tool positions, green to red by
         force against force_limit_n, at most trace_max_points points, 1 Hz, only while something subscribes),
         /mp_viz/tcp_executed (visualization_msgs/Marker, ONE thick bright-green LINE_LIST, the wall strokes only: the TCP motion actually
         executed, TF fixed_frame -> tool_frame sampled at path_rate_hz (5), kept for path_max_points samples
         (default 6000 at 1 cm steps, sampled at 5 Hz, older ones are thinned 2:1 instead of dropped, so the whole pass stays
         visible), published at path_publish_hz (1) only on change and while subscribed). This is the main trail; the force trace is the secondary one.
"""
import math
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
        max_points = int(p('trace_max_points', 1000).value)
        rate = float(p('rate_hz', 4.0).value)   # mission text; was 20 Hz (RViz redraws every message)
        self.trace = logic.TraceBuffer(max_points, float(p('trace_min_step_m', 0.002).value))
        self.force = None
        self.fb = None
        self.path_max = int(p('path_max_points', 6000).value)
        self.path_min_step = float(p('path_min_step_m', 0.01).value)   # 1 cm: the line stays smooth, 5x fewer points than 2 mm
        self.path_width = float(p('path_line_width_m', 0.02).value)
        # the green line shows the wall strokes only (the tool pressing, force above the threshold); approach, retract, stow and
        # base moves are free-space motion and drew big loops across the scene. path_contact_only:=false draws all of it again
        self.path_contact_only = bool(p('path_contact_only', True).value)
        self.path_contact_force = float(p('path_contact_force_n', 2.0).value)
        # the contact force flickers (several dropouts per second below the threshold even on a good stroke), so the line keeps
        # drawing for this long after the last sample above the threshold: one unbroken line per stroke
        self.path_contact_hold_s = float(p('path_contact_hold_s', 1.0).value)
        self._last_contact_t = None
        self.text_xyz = [float(c) for c in p('text_xyz', [0.9, -1.5, 2.4]).value]   # fixed_frame position of the readout
        self.path_pts = []
        self._path_dirty = False
        self._trace_dirty = False
        self._path_subs = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(WrenchStamped, self.force_topic, self._on_force, qos_profile_sensor_data)
        if ExecuteCoverage is not None:
            self.create_subscription(ExecuteCoverage.Impl.FeedbackMessage, self.feedback_topic, self._on_fb, 10)
        else:
            self.get_logger().warn('coverage_task_msgs not found: no mission state in the readout')
        self.text_pub = self.create_publisher(Marker, '/mp_viz/mission_text', 1)
        self.trace_pub = self.create_publisher(MarkerArray, '/mp_viz/tool_trace', 1)
        self.path_pub = self.create_publisher(Marker, '/mp_viz/tcp_executed', 1)
        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(1.0 / float(p('path_rate_hz', 5.0).value), self._sample_path)
        self.create_timer(1.0 / float(p('path_publish_hz', 1.0).value), self._publish_path)
        self.create_timer(1.0 / float(p('trace_publish_hz', 1.0).value), self._publish_trace)

    def _sample_path(self):
        try:
            t = self.tf_buffer.lookup_transform(self.fixed_frame, self.tool_frame, rclpy.time.Time(),
                                                timeout=Duration(seconds=0.0))
        except Exception:
            return
        q = t.transform.translation
        pt = (q.x, q.y, q.z)
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if self.force is not None and self.force > self.path_contact_force:
            self._last_contact_t = now_s
        in_contact = self._last_contact_t is not None and now_s - self._last_contact_t <= self.path_contact_hold_s
        if self.path_contact_only and not in_contact:
            if self.path_pts and self.path_pts[-1] is not None:
                self.path_pts.append(None)   # the stroke ended: break the line here
                self._path_dirty = True
            return
        last = self.path_pts[-1] if self.path_pts else None   # None after a break: no distance check
        if last is not None and math.dist(pt, last) < self.path_min_step:
            return
        self.path_pts.append(pt)
        self._path_dirty = True
        if len(self.path_pts) > self.path_max:  # thin the older half 2:1: the whole pass stays visible
            half = len(self.path_pts) // 2
            self.path_pts = [x for i, x in enumerate(self.path_pts[:half]) if i % 2 == 0 or x is None] + self.path_pts[half:]

    def _publish_path(self):
        subs = self.path_pub.get_subscription_count()
        if subs > self._path_subs:   # a late RViz (Volatile topic) gets the whole line on its first tick
            self._path_dirty = True
        self._path_subs = subs
        if not self._path_dirty or subs == 0:
            return
        self._path_dirty = False
        m = Marker()
        m.header.frame_id, m.header.stamp = self.fixed_frame, self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'mp_viz_tcp_executed', 0, Marker.LINE_LIST, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = self.path_width
        m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        m.points = [Point(x=x, y=y, z=z) for x, y, z in logic.line_list_points(self.path_pts)]
        self.path_pub.publish(m)

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
        if self.trace.add(pos.x, pos.y, pos.z, self.force if self.force is not None else 0.0) is not False:
            self._trace_dirty = True

        fb = self.fb
        text = Marker()
        text.header.frame_id, text.header.stamp = self.fixed_frame, now
        text.ns, text.id, text.type, text.action = 'mp_viz_text', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose.position.x, text.pose.position.y, text.pose.position.z = self.text_xyz   # fixed corner, not over the tool
        text.pose.orientation.w = 1.0
        text.scale.z = 0.07
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text.text = logic.format_readout(
            fb.state if fb else '', fb.stand_index if fb else 0, fb.stand_count if fb else 0,
            self.force, self.limit, fb.column if fb else None)
        self.text_pub.publish(text)

    def _publish_trace(self):
        # the force-coloured trace is off in RViz by default and was 61 KB x 20 Hz: only with a subscriber, 1 Hz, on change
        if not self._trace_dirty or self.trace_pub.get_subscription_count() == 0:
            return
        self._trace_dirty = False
        now = self.get_clock().now().to_msg()
        line = Marker()
        line.header.frame_id, line.header.stamp = self.fixed_frame, now
        line.ns, line.id, line.type, line.action = 'mp_viz_trace', 0, Marker.LINE_STRIP, Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.004
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
