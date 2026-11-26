"""NavigateToStand action server: drives the arm base to a stand pose with /cmd_vel, reading /odom.

Parameters: `base_profile` (robot profile YAML with a `base:` section, required), `odom_topic`,
`cmd_vel_topic`, `tolerance_xy_m`, `tolerance_yaw_rad`, `timeout_s`, `rate_hz`, `odom_timeout_s`.

Safety layer (nav_safety.py): stale odometry stops and fails with NO_ODOM; every velocity command goes through a sanity
gate (`safety_check_commands`, true; NaN/inf or over-limit -> BAD_COMMAND); no progress for `stuck_window_s` (20, 0 = off)
aborts with NAVIGATION_STUCK; `obstacle_scan_topic` (LaserScan) or `obstacle_cloud_topic` (PointCloud2), both in the base
frame, enable an obstacle stop inside a corridor ahead: `obstacle_stop_m` (0.30), `obstacle_hold_s` (5, then OBSTACLE),
`obstacle_corridor_half_width_m` (0 = footprint half width + 0.05), `obstacle_data_timeout_s` (1), `obstacle_z_min_m` /
`obstacle_z_max_m` (-0.2 / 1.5, cloud only). The reason is in the result `error_msg`.

`backend` (`builtin` | `nav2`, default builtin): nav2 forwards the goal to a Nav2 NavigateToPose action
(`nav2_action_name`, default navigate_to_pose) as a base_footprint goal; see nav2_adapter.py.
The goal frame must equal the odom frame id; nothing is transformed (no localisation here).

Path planning (path_planner.py): with a wall scene (`wall_scene_json` file and/or `wall_scene_topic`, a
std_msgs/String JSON in the odom frame) the node checks the straight drive against the scene; it keeps the
straight line when that is free (default behaviour) and plans around the obstacles when it is blocked, or always
when `use_path_planner` is true. Parameters: `use_path_planner` (false), `wall_scene_json` (''),
`wall_scene_topic` ('', e.g. /wall_detector/wall_scene), `planner_resolution_m` (0.05), `planner_margin_m` (0.02),
`planner_lookahead_m` (0 = from the profile). Without a scene, nothing changes.
"""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from stand_navigator_msgs.action import NavigateToStand

from stand_navigator.core import (
    StandController, Tolerance, arm_to_footprint, footprint_to_arm, load_profile)
from stand_navigator import nav_safety
from stand_navigator.nav_safety import SafetyConfig, SafetyMonitor
from stand_navigator.path_planner import (
    PathFollower, plan_path, scene_obstacles, straight_line_free)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class StandNavigator(Node):
    def __init__(self):
        super().__init__('stand_navigator')
        path = self.declare_parameter('base_profile', '').value
        if not path:
            raise RuntimeError('parameter base_profile (robot profile YAML) is required')
        self.profile = load_profile(path)
        self.tol = Tolerance(self.declare_parameter('tolerance_xy_m', 0.02).value,
                             self.declare_parameter('tolerance_yaw_rad', 0.02).value)
        self.timeout = self.declare_parameter('timeout_s', 120.0).value
        self.rate = self.declare_parameter('rate_hz', 50.0).value
        self.odom_timeout = self.declare_parameter('odom_timeout_s', 1.0).value
        self.use_planner = self.declare_parameter('use_path_planner', False).value
        self.res = self.declare_parameter('planner_resolution_m', 0.05).value
        self.margin = self.declare_parameter('planner_margin_m', 0.02).value
        look = self.declare_parameter('planner_lookahead_m', 0.0).value
        self.lookahead = look if look > 0.0 else None
        self.backend = self.declare_parameter('backend', 'builtin').value
        if self.backend not in ('builtin', 'nav2'):
            raise RuntimeError(f"parameter backend must be 'builtin' or 'nav2', got {self.backend!r}")
        self.nav2_client = None
        if self.backend == 'nav2':
            from stand_navigator.nav2_adapter import RosNav2Client   # needs nav2_msgs
            self.nav2_client = RosNav2Client(self, self.declare_parameter('nav2_action_name', 'navigate_to_pose').value)
        half = self.declare_parameter('obstacle_corridor_half_width_m', 0.0).value
        if half <= 0.0:
            half = max((abs(p[1]) for p in self.profile.footprint), default=0.35) + 0.05
        scan_topic = self.declare_parameter('obstacle_scan_topic', '').value
        cloud_topic = self.declare_parameter('obstacle_cloud_topic', '').value
        self.safety_cfg = SafetyConfig(
            odom_timeout_s=self.odom_timeout,
            check_commands=self.declare_parameter('safety_check_commands', True).value,
            stuck_window_s=self.declare_parameter('stuck_window_s', 20.0).value,
            stuck_min_progress=self.declare_parameter('stuck_min_progress_m', 0.02).value,
            front_stop_m=self.declare_parameter('obstacle_stop_m', 0.30).value if (scan_topic or cloud_topic) else 0.0,
            obstacle_hold_s=self.declare_parameter('obstacle_hold_s', 5.0).value,
            obstacle_data_timeout_s=self.declare_parameter('obstacle_data_timeout_s', 1.0).value,
            corridor_half_width_m=half)
        self.z_range = (self.declare_parameter('obstacle_z_min_m', -0.2).value,
                        self.declare_parameter('obstacle_z_max_m', 1.5).value)
        self.points = []
        self.points_stamp = None
        self.obstacles = []
        scene_file = self.declare_parameter('wall_scene_json', '').value
        if scene_file:
            with open(scene_file) as f:
                self.obstacles = scene_obstacles(json.load(f))
        self.odom = None
        self.odom_stamp = 0.0
        cb = ReentrantCallbackGroup()
        self.create_subscription(Odometry, self.declare_parameter('odom_topic', 'odom').value,
                                 self._on_odom, 10, callback_group=cb)
        scene_topic = self.declare_parameter('wall_scene_topic', '').value
        if scene_topic:
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(String, scene_topic, self._on_scene, latched, callback_group=cb)
        if scan_topic:
            self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data, callback_group=cb)
        if cloud_topic:
            self.create_subscription(PointCloud2, cloud_topic, self._on_cloud, qos_profile_sensor_data,
                                     callback_group=cb)
        self.pub = self.create_publisher(Twist, self.declare_parameter('cmd_vel_topic', 'cmd_vel').value, 10)
        self.server = ActionServer(
            self, NavigateToStand, 'navigate_to_stand', self._execute, callback_group=cb,
            goal_callback=self._goal, cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.get_logger().info(
            f'navigate_to_stand ready: drive {self.profile.drive}, max {self.profile.max_lin} m/s '
            f'{self.profile.max_ang} rad/s, arm mount {self.profile.arm_mount}')

    def _on_odom(self, msg):
        self.odom = msg
        self.odom_stamp = time.monotonic()

    def _on_scan(self, msg):
        self.points = nav_safety.scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                                msg.range_min, msg.range_max)
        self.points_stamp = time.monotonic()

    def _on_cloud(self, msg):
        off = {f.name: f.offset for f in msg.fields}
        if not all(k in off for k in 'xyz'):
            self.get_logger().warn('point cloud without x/y/z fields ignored')
            return
        self.points = nav_safety.cloud_to_points(bytes(msg.data), msg.point_step, (off['x'], off['y'], off['z']),
                                                 not msg.is_bigendian, self.z_range)
        self.points_stamp = time.monotonic()

    def _on_scene(self, msg):
        try:
            self.obstacles = scene_obstacles(json.loads(msg.data))
        except (ValueError, TypeError, AttributeError) as e:
            self.get_logger().warn(f'wall scene ignored: {e}')

    def _goal(self, req):
        p = req.pose.pose.position
        ok = all(map(math.isfinite, (p.x, p.y, req.pose.pose.orientation.z, req.pose.pose.orientation.w)))
        return GoalResponse.ACCEPT if ok else GoalResponse.REJECT

    def _stop(self):
        self.pub.publish(Twist())

    def _pose(self):
        o = self.odom.pose.pose
        return o.position.x, o.position.y, yaw_of(o.orientation)

    def _result(self, handle, ok, code, msg, ctl=None, arm_pose=None):
        res = NavigateToStand.Result()
        res.error_code, res.error_msg = code, msg
        if ctl is not None and self.odom is not None:
            res.position_error_m, res.yaw_error_rad = ctl.errors(self._pose())
        (handle.succeed if ok else handle.abort)()
        return res

    def _execute_nav2(self, handle):
        from stand_navigator.nav2_adapter import run_nav2_goal
        goal = handle.request.pose
        res = NavigateToStand.Result
        gp = goal.pose
        fb = NavigateToStand.Feedback()

        def feedback(st):
            fb.distance_remaining = st.distance_remaining
            handle.publish_feedback(fb)
        out = run_nav2_goal(self.nav2_client, (gp.position.x, gp.position.y, yaw_of(gp.orientation)),
                            self.profile.arm_mount, goal.header.frame_id or 'odom', self.timeout,
                            lambda: handle.is_cancel_requested, feedback)
        if out.code == 'CANCELED':
            handle.canceled()
            r = NavigateToStand.Result()
            r.error_code, r.error_msg = res.CANCELED, out.msg
            return r
        return self._result(handle, out.ok, getattr(res, out.code, res.UNKNOWN), out.msg)

    def _clearance(self, cmd):
        """Free distance ahead along the commanded direction of travel (None when not moving)."""
        if math.hypot(cmd[0], cmd[1]) < 1e-3:
            return None
        return nav_safety.corridor_clearance(self.points, math.atan2(cmd[1], cmd[0]),
                                             self.safety_cfg.corridor_half_width_m, self.safety_cfg.max_range_m)

    def _execute(self, handle):
        if self.backend == 'nav2':
            return self._execute_nav2(handle)
        goal = handle.request.pose
        res = NavigateToStand.Result
        t0 = time.monotonic()
        while self.odom is None and time.monotonic() - t0 < 2.0:
            time.sleep(0.05)
        if self.odom is None:
            return self._result(handle, False, res.NO_ODOM, 'no odometry')
        if goal.header.frame_id and goal.header.frame_id != self.odom.header.frame_id:
            return self._result(handle, False, res.INVALID_GOAL,
                                f'goal frame {goal.header.frame_id} != odom frame {self.odom.header.frame_id}')
        gp = goal.pose
        gyaw = yaw_of(gp.orientation)
        fx, fy, fyaw = arm_to_footprint(gp.position.x, gp.position.y, gyaw, self.profile.arm_mount)
        start = self._pose()
        goal_fp = (fx, fy, fyaw)
        ctl = StandController(self.profile, goal_fp, self.tol)
        note = ''
        straight = math.hypot(fx - start[0], fy - start[1])
        if self.obstacles:
            fp = self.profile.footprint
            free = straight_line_free(self.profile.holonomic, fp, start, goal_fp, self.obstacles, self.margin)
            if self.use_planner or not free:
                plan = plan_path(fp, start, goal_fp, self.obstacles, self.res, self.margin)
                if plan.ok:
                    ctl = PathFollower(self.profile, plan.path, goal_fp, self.tol, self.lookahead)
                    note = f' (planned path {plan.length:.2f} m vs straight {plan.straight:.2f} m)'
                    self.get_logger().info(f'navigate_to_stand:{note.strip()}, {len(plan.path)} waypoints')
                elif not free:
                    return self._result(handle, False, res.INVALID_GOAL,
                                        f'straight line blocked and no path: {plan.reason}')
        if not note:
            note = f' (straight line {straight:.2f} m)'
        dt = 1.0 / self.rate
        t0 = time.monotonic()
        fb = NavigateToStand.Feedback()
        last = time.monotonic()
        mon = SafetyMonitor(self.safety_cfg, last)
        sent = (0.0, 0.0, 0.0)
        heading = nav_safety.HeldDirection()
        codes = {nav_safety.NO_ODOM: res.NO_ODOM, nav_safety.BAD_COMMAND: res.BAD_COMMAND,
                 nav_safety.NAVIGATION_STUCK: res.NAVIGATION_STUCK, nav_safety.OBSTACLE: res.OBSTACLE}
        while rclpy.ok():
            if handle.is_cancel_requested:
                self._stop()
                handle.canceled()
                r = NavigateToStand.Result()
                r.error_code, r.error_msg = res.CANCELED, 'canceled'
                return r
            now = time.monotonic()
            pose = self._pose()
            dist, yerr = ctl.errors(pose)
            v = mon.check(now, self.odom_stamp, nav_safety.progress_metric(dist, yerr, self.safety_cfg),
                          dist <= self.tol.xy and abs(yerr) <= self.tol.yaw,
                          self._clearance(heading.look(sent)),
                          self.points_stamp, math.hypot(sent[0], sent[1]), self.profile.max_lin_acc)
            if v.action == nav_safety.ABORT:
                self._stop()
                self.get_logger().warn(f'navigate_to_stand aborted: {v.reason}')
                return self._result(handle, False, codes[v.code], v.reason, ctl)
            if v.action == nav_safety.HOLD:
                heading.hold(sent)
                self._stop()
                sent = (0.0, 0.0, 0.0)
                ctl.cmd = getattr(ctl, 'ctl', ctl).cmd = sent
                last = now
                time.sleep(dt)
                continue
            if now - t0 > self.timeout:
                self._stop()
                return self._result(handle, False, res.TIMEOUT, f'not there after {self.timeout:.0f} s', ctl)
            tw = self.odom.twist.twist
            if ctl.arrived(pose, math.hypot(tw.linear.x, tw.linear.y), abs(tw.angular.z)):
                self._stop()
                return self._result(handle, True, res.NONE, 'arrived' + note, ctl)
            step_dt = max(now - last, 1e-3)
            vx, vy, wz = ctl.step(pose, step_dt)
            last = now
            if self.safety_cfg.check_commands:
                (vx, vy, wz), v = nav_safety.gate_command((vx, vy, wz), sent, step_dt, self.profile, self.safety_cfg)
                if v.action == nav_safety.ABORT:
                    self._stop()
                    self.get_logger().warn(f'navigate_to_stand aborted: {v.reason}')
                    return self._result(handle, False, res.BAD_COMMAND, v.reason, ctl)
            sent = (vx, vy, wz)
            cmd = Twist()
            cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
            self.pub.publish(cmd)
            ax, ay, ayaw = footprint_to_arm(*pose, self.profile.arm_mount)
            fb.distance_remaining, fb.yaw_remaining = ctl.errors(pose)
            fb.current_pose = PoseStamped()
            fb.current_pose.header = self.odom.header
            fb.current_pose.pose.position.x, fb.current_pose.pose.position.y = ax, ay
            fb.current_pose.pose.orientation.z, fb.current_pose.pose.orientation.w = (
                math.sin(ayaw / 2), math.cos(ayaw / 2))
            handle.publish_feedback(fb)
            time.sleep(dt)
        self._stop()
        return self._result(handle, False, res.UNKNOWN, 'shutdown', ctl)


def main():
    rclpy.init()
    node = StandNavigator()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()


if __name__ == '__main__':
    main()
