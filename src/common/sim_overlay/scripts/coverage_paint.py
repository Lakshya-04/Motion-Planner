#!/usr/bin/env python3
"""Live done / not-done sweep paint of the wall (RViz, and through the existing overlay nodes Gazebo and MuJoCo).

Reads:  TF (tcp_frame in the wall frame), the contact force (WrenchStamped), a wall json, optionally the plan file (column
        geometry for failed columns) and the ExecuteCoverage feedback (topic <execute_coverage>/_action/feedback).
Writes: /mp_viz/coverage_paint (MarkerArray, transient_local: NOT_DONE red, DONE green, FAILED orange, wall outline,
        percent text), /mp_viz/coverage_percent (std_msgs/Float32).
Services (std_srvs/Trigger): ~/reset, ~/save_png (to parameter png_path).
Topic ~/outcome (std_msgs/String, JSON {"done": [..], "partial": [..], "failed": [..]}): column outcome from the action
result (the result is not a topic; a client can publish it here, the live inference from feedback needs nothing).

Parameters: wall_scene_json (required), wall_id 0, frame_id ('' = the json's frame_id, else 'odom'), tcp_frame tcp_link,
  source mujoco|gazebo|custom (force topic /sim_mujoco/ft_ground_truth | /ft_sensor/ground_truth_wrench), force_topic,
  force_msg wrench|info (info = Float64MultiArray /ft_sensor/ground_truth_info, uses max_point_norm), force_mode z|norm,
  cell_m 0.02, swath_m 0.14, force_threshold_n 1.0 (<= 0 ignores force), plane_tol_m 0.03, sample_hz 50, publish_hz 1,
  plan_file '', feedback_topic, png_path /tmp/coverage_paint.png, lift_m 0.003 (offset in front of the wall).
Gazebo / MuJoCo: start gz_overlay.py / overlay_dump.py with  -p topics:="['/mp_viz/coverage_paint']".
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import rclpy  # noqa: E402
from geometry_msgs.msg import Point, WrenchStamped  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data  # noqa: E402
from std_msgs.msg import ColorRGBA, Float32, Float64MultiArray, String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

from sim_overlay import coverage_paint as cp  # noqa: E402

SOURCES = {'mujoco': ('/sim_mujoco/ft_ground_truth', 'wrench'), 'gazebo': ('/ft_sensor/ground_truth_wrench', 'wrench')}
TYPES = {'TRIANGLE_LIST': Marker.TRIANGLE_LIST, 'LINE_STRIP': Marker.LINE_STRIP, 'TEXT_VIEW_FACING': Marker.TEXT_VIEW_FACING}


class CoveragePaint(Node):
    def __init__(self):
        super().__init__('coverage_paint')
        p = lambda k, d: self.declare_parameter(k, d).value  # noqa: E731
        wall = cp.load_wall(p('wall_scene_json', ''), int(p('wall_id', 0)))
        self.frame = p('frame_id', '') or wall.frame_id or 'odom'
        wall.frame_id = self.frame
        self.grid = cp.CoverageGrid(wall, p('cell_m', 0.02), p('swath_m', 0.14), p('force_threshold_n', 1.0),
                                    p('plane_tol_m', 0.03))
        src = p('source', 'mujoco')
        topic, msg = SOURCES.get(src, ('', 'wrench'))
        topic, msg = p('force_topic', '') or topic, p('force_msg', '') or msg
        self.mode, self.tcp, self.lift = p('force_mode', 'z'), p('tcp_frame', 'tcp_link'), float(p('lift_m', 0.003))
        self.png = p('png_path', '/tmp/coverage_paint.png')
        self.force = 0.0
        plan = p('plan_file', '')
        if plan:
            self.grid.set_columns(cp.load_plan_columns(plan))
            self.get_logger().info('plan columns: %d' % len(self.grid.columns))
        self.tracker = cp.FeedbackTracker()
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        if topic:
            if msg == 'info':
                self.create_subscription(Float64MultiArray, topic, self._info, qos_profile_sensor_data)
            else:
                self.create_subscription(WrenchStamped, topic, self._wrench, qos_profile_sensor_data)
        fb = p('feedback_topic', '/execute_coverage/_action/feedback')
        if fb:
            try:
                from coverage_task_msgs.action import ExecuteCoverage
                self.create_subscription(ExecuteCoverage.Impl.FeedbackMessage, fb, self._feedback, 10)
            except ImportError:
                self.get_logger().warn('coverage_task_msgs not found: no feedback marking (use ~/outcome)')
        self.create_subscription(String, '~/outcome', self._outcome, 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(MarkerArray, '/mp_viz/coverage_paint', qos)
        self.pct = self.create_publisher(Float32, '/mp_viz/coverage_percent', 10)
        self.create_service(Trigger, '~/reset', self._reset)
        self.create_service(Trigger, '~/save_png', self._save)
        self.create_timer(1.0 / float(p('sample_hz', 50.0)), self._sample)
        self.create_timer(1.0 / float(p('publish_hz', 1.0)), self._publish)
        self.get_logger().info('coverage paint: %dx%d cells of %.3f m, swath %.2f m, force >= %.1f N, topic %s (%s)' % (
            self.grid.nu, self.grid.nv, self.grid.cell, self.grid.swath, self.grid.force_threshold, topic, msg))

    def _wrench(self, m):
        f = m.wrench.force
        self.force = abs(f.z) if self.mode == 'z' else math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)

    def _info(self, m):
        self.force = float(m.data[4]) if len(m.data) >= 5 else 0.0   # max_point_norm

    def _sample(self):
        try:
            t = self.tf.lookup_transform(self.frame, self.tcp, rclpy.time.Time()).transform.translation
        except Exception:  # noqa: BLE001
            return
        self.grid.add_sample((t.x, t.y, t.z), self.force, self.get_clock().now().nanoseconds * 1e-9)

    def _feedback(self, msg):
        f = msg.feedback
        for col, what in self.tracker.update(int(f.column), int(f.columns_done), int(f.columns_failed)):
            if what == 'failed':
                self.grid.mark_column(col, cp.FAILED)

    def _outcome(self, msg):
        try:
            d = json.loads(msg.data)
            self.grid.mark_outcome(d.get('done', []), d.get('partial', []), d.get('failed', []))
        except (ValueError, AttributeError) as e:
            self.get_logger().warn('outcome json: %s' % e)

    def _publish(self):
        if not self.grid.dirty:
            return
        self.grid.dirty = False
        arr = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for d in self.grid.marker_dicts(self.lift):
            m = Marker(type=TYPES[d['type']], action=Marker.ADD if d['action'] == 'ADD' else Marker.DELETE,
                       ns=d['ns'], id=d['id'])
            m.header.frame_id = self.frame
            m.pose.orientation.w = 1.0
            m.pose.position.x, m.pose.position.y, m.pose.position.z = d['pose'][:3]
            m.scale.x, m.scale.y, m.scale.z = d['scale']
            m.color = ColorRGBA(r=float(d['color'][0]), g=float(d['color'][1]), b=float(d['color'][2]), a=float(d['color'][3]))
            m.points = [Point(x=x, y=y, z=z) for x, y, z in d['points']]
            m.text = d['text']
            arr.markers.append(m)
        self.pub.publish(arr)
        self.pct.publish(Float32(data=float(self.grid.percent_done())))
        self.get_logger().info('covered %.1f %% %s' % (self.grid.percent_done(), self.grid.counts()), throttle_duration_sec=10.0)

    def _reset(self, _req, res):
        self.grid.status[:] = 0
        self.grid.break_stroke()
        self.grid.dirty = True
        res.success, res.message = True, 'reset'
        return res

    def _save(self, _req, res):
        try:
            cp.render_png(self.grid, self.png, 'live')
            res.success, res.message = True, self.png
        except Exception as e:  # noqa: BLE001
            res.success, res.message = False, str(e)
        return res


def main():
    rclpy.init()
    node = CoveragePaint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
