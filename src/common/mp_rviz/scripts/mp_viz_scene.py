#!/usr/bin/env python3
"""Scene markers no module publishes itself: the planned wall columns and the base stands.

Reads:   coverage_topic (coverage_planner_msgs/CoveragePlan; default /wall_coverage_service_node/coverage_plan),
         service plan_stands_service (reachability_msgs/PlanStands; default /reachability_node/plan_stands),
         std_srvs/Trigger ~/refresh: calls PlanStands once with the last plan's path poses as targets_world.
Writes:  /mp_viz/plan_columns (MarkerArray, one LINE_STRIP per segment; connectors grey),
         /mp_viz/stands (MarkerArray, per stand an ARROW, a footprint LINE_STRIP and a TEXT_VIEW_FACING),
         /mp_viz/reach_slice (MarkerArray; reserved: reachability_msgs has no map/slice service, nothing is published).
All three are transient_local so a late RViz gets the last state.
The wall plane for PlanStands comes from parameters wall_point / wall_normal (3 floats each, an empty list = derive
from the first target: its point, and the opposite of its tool z axis); max_targets caps the request size.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402
from geometry_msgs.msg import Point, Pose  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import ColorRGBA  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

import mp_viz_scene_logic as logic  # noqa: E402

try:
    from coverage_planner_msgs.msg import CoveragePlan
except ImportError:
    CoveragePlan = None
try:
    from reachability_msgs.srv import PlanStands
except ImportError:
    PlanStands = None


def _color(rgb, a=1.0):
    return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(a))


class MpVizScene(Node):
    def __init__(self):
        super().__init__('mp_viz_scene')
        p = self.declare_parameter
        coverage_topic = p('coverage_topic', '/wall_coverage_service_node/coverage_plan').value
        stands_srv = p('plan_stands_service', '/reachability_node/plan_stands').value
        self.include_connectors = bool(p('include_connectors', True).value)
        self.max_targets = int(p('max_targets', 400).value)
        self.fp_len = float(p('footprint_length', 0.8).value)
        self.fp_wid = float(p('footprint_width', 0.6).value)
        self.wall_point = list(p('wall_point', [0.0]).value)
        self.wall_normal = list(p('wall_normal', [0.0]).value)
        self.frame = p('frame_id', '').value
        self.plan = None

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.cols_pub = self.create_publisher(MarkerArray, '/mp_viz/plan_columns', qos)
        self.stands_pub = self.create_publisher(MarkerArray, '/mp_viz/stands', qos)
        self.reach_pub = self.create_publisher(MarkerArray, '/mp_viz/reach_slice', qos)
        self.reach_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
        if CoveragePlan is not None:
            self.create_subscription(CoveragePlan, coverage_topic, self._on_plan, qos)
        else:
            self.get_logger().warn('coverage_planner_msgs not found: no plan columns')
        if PlanStands is not None:
            self.client = self.create_client(PlanStands, stands_srv)
        else:
            self.client = None
            self.get_logger().warn('reachability_msgs not found: no stands')
        self.create_service(Trigger, '~/refresh', self._on_refresh)

    # plan columns
    def _on_plan(self, msg):
        self.plan = msg
        pts = [(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z) for ps in msg.path.poses]
        segs = [{'label': s.label, 'mode': s.mode, 'start_index': s.start_index, 'end_index': s.end_index,
                 'render_start_index': s.render_start_index, 'render_end_index': s.render_end_index}
                for s in msg.segments]
        lines = logic.segment_polylines(segs, pts, self.include_connectors) or logic.path_fallback(pts)
        frame = self.frame or msg.path.header.frame_id or 'world'
        arr = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for i, ln in enumerate(lines):
            m = Marker(type=Marker.LINE_STRIP, action=Marker.ADD, ns='plan_columns', id=i)
            m.header.frame_id = frame
            m.pose.orientation.w = 1.0
            m.scale.x = 0.01 if ln['connector'] else 0.025
            m.color = _color(ln['rgb'], 0.5 if ln['connector'] else 1.0)
            m.points = [Point(x=x, y=y, z=z) for x, y, z in ln['points']]
            arr.markers.append(m)
        self.cols_pub.publish(arr)
        self.get_logger().info('plan: %d line(s) from %d poses' % (len(lines), len(pts)))

    # stands
    def _on_refresh(self, _req, res):
        if self.client is None or self.plan is None or not self.plan.path.poses:
            res.success, res.message = False, 'need a PlanStands client and a received coverage plan'
            return res
        if not self.client.service_is_ready():
            res.success, res.message = False, 'PlanStands service not available'
            return res
        poses = logic.subsample([ps.pose for ps in self.plan.path.poses], self.max_targets)
        req = PlanStands.Request()
        req.targets_world = list(poses)
        first = poses[0]
        q = (first.orientation.x, first.orientation.y, first.orientation.z, first.orientation.w)
        wp = self.wall_point if len(self.wall_point) == 3 else [first.position.x, first.position.y, first.position.z]
        wn = self.wall_normal if len(self.wall_normal) == 3 else list(logic.wall_normal_from_tool(q))
        req.wall_point = Point(x=float(wp[0]), y=float(wp[1]), z=float(wp[2]))
        req.wall_normal.x, req.wall_normal.y, req.wall_normal.z = (float(c) for c in wn)
        fut = self.client.call_async(req)
        fut.add_done_callback(self._on_stands)
        res.success, res.message = True, 'PlanStands requested with %d targets' % len(poses)
        return res

    def _on_stands(self, fut):
        r = fut.result()
        if r is None or not r.stands:
            self.get_logger().warn('PlanStands: no stands (%s)' % (r.message if r else 'no response'))
            return
        frame = self.frame or (self.plan.path.header.frame_id if self.plan else '') or 'world'
        stands = [{'x': s.x, 'y': s.y, 'yaw': s.yaw, 'travel_s': s.travel_s,
                   'target_indices': list(s.target_indices)} for s in r.stands]
        arr = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for sp, s in zip(logic.stand_specs(stands), r.stands):
            i = sp['index']
            base = dict(action=Marker.ADD, id=i)

            def hdr(m):
                m.header.frame_id = frame
                return m
            a = hdr(Marker(type=Marker.ARROW, ns='stand_arrow', **base))
            a.pose.position.x, a.pose.position.y = s.x, s.y
            a.pose.orientation.x, a.pose.orientation.y, a.pose.orientation.z, a.pose.orientation.w = sp['quat']
            a.scale.x, a.scale.y, a.scale.z = 0.5, 0.06, 0.06
            a.color = _color(sp['rgb'])
            f = hdr(Marker(type=Marker.LINE_STRIP, ns='stand_footprint', **base))
            f.pose.orientation.w = 1.0
            f.scale.x = 0.02
            f.color = _color(sp['rgb'])
            f.points = [Point(x=x, y=y, z=z)
                        for x, y, z in logic.footprint_corners(s.x, s.y, s.yaw, self.fp_len, self.fp_wid)]
            t = hdr(Marker(type=Marker.TEXT_VIEW_FACING, ns='stand_text', **base))
            t.pose.position.x, t.pose.position.y, t.pose.position.z = s.x, s.y, 0.4
            t.pose.orientation.w = 1.0
            t.scale.z = 0.15
            t.color = _color((1.0, 1.0, 1.0))
            t.text = sp['label']
            arr.markers += [a, f, t]
        self.stands_pub.publish(arr)
        self.get_logger().info('stands: %d' % len(r.stands))


def main():
    rclpy.init()
    node = MpVizScene()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
