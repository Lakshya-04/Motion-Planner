"""NavigateToStand action server: drives the arm base to a stand pose with /cmd_vel, reading /odom.

Parameters: `base_profile` (robot profile YAML with a `base:` section, required), `odom_topic`,
`cmd_vel_topic`, `tolerance_xy_m`, `tolerance_yaw_rad`, `timeout_s`, `rate_hz`, `odom_timeout_s`.
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from stand_navigator_msgs.action import NavigateToStand

from stand_navigator.core import (
    StandController, Tolerance, arm_to_footprint, footprint_to_arm, load_profile)
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

    def _execute(self, handle):
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
        while rclpy.ok():
            if handle.is_cancel_requested:
                self._stop()
                handle.canceled()
                r = NavigateToStand.Result()
                r.error_code, r.error_msg = res.CANCELED, 'canceled'
                return r
            now = time.monotonic()
            if now - self.odom_stamp > self.odom_timeout:
                self._stop()
                return self._result(handle, False, res.NO_ODOM, 'odometry went stale', ctl)
            if now - t0 > self.timeout:
                self._stop()
                return self._result(handle, False, res.TIMEOUT, f'not there after {self.timeout:.0f} s', ctl)
            pose = self._pose()
            tw = self.odom.twist.twist
            if ctl.arrived(pose, math.hypot(tw.linear.x, tw.linear.y), abs(tw.angular.z)):
                self._stop()
                return self._result(handle, True, res.NONE, 'arrived' + note, ctl)
            vx, vy, wz = ctl.step(pose, max(now - last, 1e-3))
            last = now
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
