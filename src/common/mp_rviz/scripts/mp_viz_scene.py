#!/usr/bin/env python3
"""Scene markers no module publishes itself: the planned wall columns and the base stands.

Reads:   coverage_topic (coverage_planner_msgs/CoveragePlan; default /wall_coverage_service_node/coverage_plan),
         service plan_stands_service (reachability_msgs/PlanStands; default /reachability/plan_stands),
         std_srvs/Trigger ~/refresh: calls PlanStands once with the last plan's path poses as targets_world.
         parameter plan_file: a coverage_output.json (the file the manager executes: final_lifted_and_expanded, else
         the top-level path/segments) read at start and on ~/refresh; with it no CoveragePlan topic is needed. Set
         frame_id to the frame the file's poses are meant in (the manager's world_frame, e.g. odom), the file's own
         frame_id (e.g. base_link) is only right while the base is at that frame's origin.
         plan_file and wall_scene_json are re-read when their modification time changes (checked every 2 s).
Writes:  /mp_viz/plan_columns (MarkerArray, one LINE_STRIP per segment; connectors grey),
         /mp_viz/stands (MarkerArray, per stand an ARROW, a footprint LINE_STRIP and a TEXT_VIEW_FACING),
         /mp_viz/reach_slice (MarkerArray; reserved: reachability_msgs has no map/slice service, nothing is published).
         /mp_viz/wall (MarkerArray: translucent quad + outline + text of the real wall from parameter wall_scene_json,
         wall id wall_id; frame_id param or the scene's frame_id),
         /mp_viz/reachable (MarkerArray: a CUBE_LIST of green = IsReachable true, red = false, one cube per sampled plan
         pose, plus a text with the count). Computed on ~/refresh with reachability_msgs/IsReachable, one async request
         per pose (at most max_reach_targets, evenly sampled), for a base at base_x/base_y/base_yaw (frame_id frame) and
         the arm base at height arm_base_z (+ arm_dx/arm_dy in the base frame). ONE lift height only.
         /mp_viz/stand_columns (MarkerArray: per stand a POINTS marker in that stand's colour at the plan poses
         PlanStands assigned to it = the columns reachable from that stand, plus a text 'stand i: n poses').
         /mp_viz/transitions (MarkerArray: grey arrows from the end of each stroke to the start of the next, in execution
         order, plus the visiting number at the start of each stroke group; drawn from start_index/end_index).
All topics are transient_local so a late RViz gets the last state.
The wall plane for PlanStands comes from parameters wall_point / wall_normal (3 floats each, an empty list = derive
from the first target: its point, and the opposite of its tool z axis); max_targets caps the request size.
"""
import json
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
    from reachability_msgs.srv import IsReachable, PlanStands
except ImportError:
    IsReachable = PlanStands = None


def _color(rgb, a=1.0):
    return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(a))


class MpVizScene(Node):
    def __init__(self):
        super().__init__('mp_viz_scene')
        p = self.declare_parameter
        coverage_topic = p('coverage_topic', '/wall_coverage_service_node/coverage_plan').value
        stands_srv = p('plan_stands_service', '/reachability/plan_stands').value
        self.include_connectors = bool(p('include_connectors', True).value)
        self.max_targets = int(p('max_targets', 400).value)
        self.fp_len = float(p('footprint_length', 0.8).value)
        self.fp_wid = float(p('footprint_width', 0.6).value)
        self.wall_point = list(p('wall_point', [0.0]).value)
        self.wall_normal = list(p('wall_normal', [0.0]).value)
        self.frame = p('frame_id', '').value
        self.plan = None
        self.plan_file = p('plan_file', '').value
        self.wall_scene_json = p('wall_scene_json', '').value
        self.wall_id = int(p('wall_id', 0).value)
        self.strip_width = float(p('strip_width', 0.1).value)
        is_reach_srv = p('is_reachable_service', '/reachability/is_reachable').value
        self.max_reach = int(p('max_reach_targets', 150).value)
        self.reach_step = float(p('reach_step', 0.15).value)
        self.base = (float(p('base_x', 0.0).value), float(p('base_y', 0.0).value), float(p('base_yaw', 0.0).value))
        self.arm_base_z = float(p('arm_base_z', 1.0).value)
        self.arm_d = (float(p('arm_dx', 0.0).value), float(p('arm_dy', 0.0).value))

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.cols_pub = self.create_publisher(MarkerArray, '/mp_viz/plan_columns', qos)
        self.stands_pub = self.create_publisher(MarkerArray, '/mp_viz/stands', qos)
        self.reach_pub = self.create_publisher(MarkerArray, '/mp_viz/reach_slice', qos)
        self.reach_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
        self.trans_pub = self.create_publisher(MarkerArray, '/mp_viz/transitions', qos)
        self.wall_pub = self.create_publisher(MarkerArray, '/mp_viz/wall', qos)
        self.stand_cols_pub = self.create_publisher(MarkerArray, '/mp_viz/stand_columns', qos)
        self.reachable_pub = self.create_publisher(MarkerArray, '/mp_viz/reachable', qos)
        self.reach_client = self.create_client(IsReachable, is_reach_srv) if IsReachable is not None else None
        self._reach_gen = 0
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
        self._publish_wall()
        if self.plan_file:
            self._load_file()
        # the plan / wall files are often written after this node started (the coverage run): reload on change
        self._mtimes = {}
        self.create_timer(2.0, self._watch_files)

    def _watch_files(self):
        for path, fn in ((self.plan_file, self._load_file), (self.wall_scene_json, self._publish_wall)):
            if not path:
                continue
            try:
                m = os.stat(path).st_mtime
            except OSError:
                continue
            if self._mtimes.get(path) != m:
                self._mtimes[path] = m
                fn()

    # wall
    def _publish_wall(self):
        if not self.wall_scene_json:
            return False
        try:
            with open(self.wall_scene_json) as f:
                scene = json.load(f)
            wall = logic.wall_from_scene(scene, self.wall_id)
        except (OSError, ValueError, KeyError) as e:
            self.get_logger().error('wall_scene_json %s: %s' % (self.wall_scene_json, e))
            return False
        self._wall = wall
        frame = self.frame or scene.get('frame_id') or 'map'
        arr = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        q = Marker(type=Marker.TRIANGLE_LIST, action=Marker.ADD, ns='wall_quad', id=0)
        q.header.frame_id = frame
        q.pose.orientation.w = 1.0
        q.scale.x = q.scale.y = q.scale.z = 1.0
        q.color = _color((0.3, 0.55, 0.95), 0.25)
        q.points = [Point(x=x, y=y, z=z) for x, y, z in logic.wall_quad_triangles(wall)]
        o = Marker(type=Marker.LINE_STRIP, action=Marker.ADD, ns='wall_outline', id=0)
        o.header.frame_id = frame
        o.pose.orientation.w = 1.0
        o.scale.x = 0.03
        o.color = _color((0.3, 0.55, 0.95), 1.0)
        o.points = [Point(x=x, y=y, z=z) for x, y, z in logic.wall_outline(wall)]
        t = Marker(type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, ns='wall_text', id=0)
        t.header.frame_id = frame
        c = logic.wall_corners(wall)
        t.pose.position.x, t.pose.position.y, t.pose.position.z = (
            sum(p[k] for p in c) / 4.0 for k in range(3))
        t.pose.orientation.w = 1.0
        t.scale.z = 0.15
        t.color = _color((1.0, 1.0, 1.0))
        t.text = 'wall %.1f x %.1f m = %.1f m2' % (
            abs(wall['u_range_m'][1] - wall['u_range_m'][0]), abs(wall['v_range_m'][1] - wall['v_range_m'][0]),
            logic.wall_area_m2(wall))
        arr.markers += [q, o, t]
        self.wall_pub.publish(arr)
        self.get_logger().info('wall layer: %s' % t.text)
        return True

    # reachable
    def _request_reachable(self):
        """Async IsReachable for up to max_reach sampled plan poses; publishes when all answers are in."""
        if self.reach_client is None or not getattr(self, '_poses', None):
            return 'no IsReachable client or no plan'
        if not self.reach_client.service_is_ready():
            return 'IsReachable service not available'
        tuples = [(q.position.x, q.position.y, q.position.z, q.orientation.x, q.orientation.y, q.orientation.z,
                   q.orientation.w) for q in self._poses]
        ranges = [(a, b) for a, b in getattr(self, '_stroke_ranges', []) if 0 <= a < b < len(tuples)]
        if ranges:
            tuples = logic.densify_poses(tuples, ranges, self.reach_step)
        poses = []
        for t in logic.subsample(tuples, self.max_reach):
            ps = Pose()
            ps.position.x, ps.position.y, ps.position.z = t[:3]
            ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = t[3:]
            poses.append(ps)
        self._reach_gen += 1
        gen, results = self._reach_gen, [None] * len(poses)
        frame = self.frame or getattr(self, '_file_frame', '') or 'world'
        bx, by, byaw = self.base
        # One request in flight at a time: with 150 requests queued at once the reachability node answered about one
        # per second (measured), one after the other it answers in milliseconds.
        def send(i):
            ps = poses[i]
            wp = (ps.position.x, ps.position.y, ps.position.z,
                  ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w)
            lp = logic.world_to_arm_base(wp, bx, by, byaw, self.arm_base_z, *self.arm_d)
            req = IsReachable.Request()
            (req.tool_in_arm_base.position.x, req.tool_in_arm_base.position.y, req.tool_in_arm_base.position.z) = lp[:3]
            (req.tool_in_arm_base.orientation.x, req.tool_in_arm_base.orientation.y,
             req.tool_in_arm_base.orientation.z, req.tool_in_arm_base.orientation.w) = lp[3:]
            self.reach_client.call_async(req).add_done_callback(lambda f, i=i: done(f, i))

        def done(f, i):
            try:
                if gen != self._reach_gen:
                    return
                r = f.result()
                results[i] = bool(r.reachable) if r is not None else False
                if i + 1 < len(poses):
                    send(i + 1)
                else:
                    self._publish_reachable(poses, results, frame)
            except Exception as e:  # rclpy drops exceptions of done callbacks silently
                self.get_logger().error('IsReachable callback: %r' % (e,))
        send(0)
        return 'IsReachable requested for %d poses' % len(poses)

    def _publish_reachable(self, poses, flags, frame):
        arr = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for name, want, idx in (('reachable', True, 0), ('unreachable', False, 1)):
            m = Marker(type=Marker.CUBE_LIST, action=Marker.ADD, ns=name, id=idx)
            m.header.frame_id = frame
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.04
            m.color = _color(logic.reach_colour(want), 0.9)
            m.points = [Point(x=ps.position.x, y=ps.position.y, z=ps.position.z)
                        for ps, f in zip(poses, flags) if f == want]
            arr.markers.append(m)
        t = Marker(type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, ns='reachable_text', id=2)
        t.header.frame_id = frame
        t.pose.position.x, t.pose.position.y, t.pose.position.z = (
            self.base[0], self.base[1], self.arm_base_z + 1.2)
        t.pose.orientation.w = 1.0
        t.scale.z = 0.15
        t.color = _color((1.0, 1.0, 1.0))
        t.text = 'base (%.2f, %.2f, yaw %.2f): %s' % (*self.base, logic.reach_summary(flags))
        arr.markers.append(t)
        self.reachable_pub.publish(arr)
        self.get_logger().info('reachable: ' + t.text)

    # plan columns
    def _on_plan(self, msg):
        self.plan = msg
        pts = [(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z) for ps in msg.path.poses]
        segs = [{'label': s.label, 'mode': s.mode, 'start_index': s.start_index, 'end_index': s.end_index,
                 'render_start_index': s.render_start_index, 'render_end_index': s.render_end_index}
                for s in msg.segments]
        self._poses = [ps.pose for ps in msg.path.poses]
        self._publish_plan(pts, segs, msg.path.header.frame_id)

    def _load_file(self):
        try:
            with open(self.plan_file) as f:
                d = json.load(f)
        except (OSError, ValueError) as e:
            self.get_logger().error('plan_file %s: %s' % (self.plan_file, e))
            return False
        r = d.get('final_lifted_and_expanded') or d
        path = r.get('path') or {}
        poses = path.get('poses') or []
        pts = [(q['position']['x'], q['position']['y'], q['position']['z']) for q in poses]
        self._poses = []
        for q in poses:
            ps = Pose()
            ps.position.x, ps.position.y, ps.position.z = (q['position'][k] for k in 'xyz')
            o = q['orientation']
            ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = (o[k] for k in 'xyzw')
            self._poses.append(ps)
        self._file_frame = path.get('frame_id', '')
        self._stroke_ranges = [(int(sg.get('render_start_index', sg.get('start_index', 0))),
                                int(sg.get('render_end_index', sg.get('end_index', 0))))
                               for sg in (r.get('segments') or [])
                               if not logic.is_connector(sg.get('label', ''), sg.get('mode', ''))]
        self.plan = True
        self._publish_plan(pts, r.get('segments') or [], self._file_frame)
        return True

    def _publish_plan(self, pts, segs, msg_frame):
        lines = logic.segment_polylines(segs, pts, self.include_connectors) or logic.path_fallback(pts)
        frame = self.frame or msg_frame or 'world'
        arr = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for i, ln in enumerate(lines):
            m = Marker(type=Marker.LINE_STRIP, action=Marker.ADD, ns='plan_columns', id=i)
            m.header.frame_id = frame
            m.pose.orientation.w = 1.0
            m.scale.x = 0.01 if ln['connector'] else 0.03
            m.color = _color(ln['rgb'], 0.5 if ln['connector'] else 1.0)
            m.points = [Point(x=x, y=y, z=z) for x, y, z in ln['points']]
            arr.markers.append(m)
        self._publish_transitions(segs, pts, frame)
        w = getattr(self, '_wall', None)
        if w:
            self.get_logger().info('plan strokes ~%.2f m2 (strip %.2f m) of the wall %.2f m2' % (
                logic.plan_strip_area_m2(lines, self.strip_width), self.strip_width, logic.wall_area_m2(w)))
        self.cols_pub.publish(arr)
        self.get_logger().info('plan: %d line(s) from %d poses in frame %s' % (len(lines), len(pts), frame))

    def _publish_transitions(self, segs, pts, frame):
        joins, nums = logic.transitions(segs, pts)
        arr = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for i, j in enumerate(joins):
            m = Marker(type=Marker.ARROW, action=Marker.ADD, ns='transition', id=i)
            m.header.frame_id = frame
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = 0.012, 0.035, 0.05
            m.color = _color((0.75, 0.75, 0.75), 0.9)
            m.points = [Point(x=float(c[0]), y=float(c[1]), z=float(c[2])) for c in (j['a'], j['b'])]
            arr.markers.append(m)
        for k, nn in enumerate(nums):
            t = Marker(type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, ns='visit_order', id=k)
            t.header.frame_id = frame
            t.pose.position.x, t.pose.position.y, t.pose.position.z = (
                float(nn['at'][0]), float(nn['at'][1]), float(nn['at'][2]) + 0.06)
            t.pose.orientation.w = 1.0
            t.scale.z = 0.07
            t.color = _color((1.0, 1.0, 0.4))
            t.text = str(nn['n'])
            arr.markers.append(t)
        self.trans_pub.publish(arr)

    # stands
    def _on_refresh(self, _req, res):
        if self.plan_file:
            self._load_file()
        self._publish_wall()
        reach_msg = self._request_reachable()
        self.get_logger().info('refresh: ' + reach_msg)
        if self.client is None or self.plan is None or not getattr(self, '_poses', None):
            res.success, res.message = False, 'need a PlanStands client and a plan (topic or plan_file); ' + reach_msg
            return res
        if not self.client.service_is_ready():
            res.success, res.message = False, 'PlanStands service not available; ' + reach_msg
            return res
        poses = logic.subsample(list(self._poses), self.max_targets)
        req = PlanStands.Request()
        req.targets_world = list(poses)
        first = poses[0]
        q = (first.orientation.x, first.orientation.y, first.orientation.z, first.orientation.w)
        wp = self.wall_point if len(self.wall_point) == 3 else [first.position.x, first.position.y, first.position.z]
        wn = self.wall_normal if len(self.wall_normal) == 3 else list(logic.wall_normal_from_tool(q))
        req.wall_point = Point(x=float(wp[0]), y=float(wp[1]), z=float(wp[2]))
        req.wall_normal.x, req.wall_normal.y, req.wall_normal.z = (float(c) for c in wn)
        self._stand_poses = poses
        fut = self.client.call_async(req)
        fut.add_done_callback(self._on_stands)
        res.success, res.message = True, 'PlanStands requested with %d targets; %s' % (len(poses), reach_msg)
        return res

    def _on_stands(self, fut):
        r = fut.result()
        if r is None or not r.stands:
            self.get_logger().warn('PlanStands: no stands (%s)' % (r.message if r else 'no response'))
            return
        frame = self.frame or getattr(self, '_file_frame', '') or (self.plan.path.header.frame_id if hasattr(self.plan, 'path') else '') or 'world'
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
        self._publish_stand_columns(r.stands, frame)
        self.get_logger().info('stands: %d' % len(r.stands))

    def _publish_stand_columns(self, stands, frame):
        poses = getattr(self, '_stand_poses', None) or []
        arr = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for i, s in enumerate(stands):
            idx = [k for k in s.target_indices if 0 <= k < len(poses)]
            rgb = logic.palette(i)
            m = Marker(type=Marker.POINTS, action=Marker.ADD, ns='stand_columns', id=i)
            m.header.frame_id = frame
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.04
            m.color = _color(rgb, 1.0)
            m.points = [Point(x=poses[k].position.x, y=poses[k].position.y, z=poses[k].position.z) for k in idx]
            arr.markers.append(m)
            if idx:
                t = Marker(type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, ns='stand_columns_text', id=i)
                t.header.frame_id = frame
                t.pose.position.x, t.pose.position.y = s.x, s.y
                t.pose.position.z = 0.7
                t.pose.orientation.w = 1.0
                t.scale.z = 0.12
                t.color = _color(rgb)
                t.text = 'stand %d reaches %d poses' % (i, len(idx))
                arr.markers.append(t)
        self.stand_cols_pub.publish(arr)


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
