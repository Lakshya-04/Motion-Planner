"""Optional Nav2 backend for navigate_to_stand (`backend:=nav2`).

The arm-base stand pose is converted to a base_footprint goal with `core.arm_to_footprint` and forwarded to a Nav2
`NavigateToPose` action client, which sits behind the `Nav2Client` interface so the logic is tested with a fake
and the package works without Nav2 installed: `nav2_msgs` is imported only inside `RosNav2Client`.

`run_nav2_goal` is the ROS-free loop; it returns an `Outcome` whose `code` is the NAME of a NavigateToStand error
code (NONE, UNKNOWN, TIMEOUT, CANCELED, NO_NAV2), mapped to the number by the node.
"""
import math
import time
from dataclasses import dataclass

from stand_navigator.core import arm_to_footprint

ACTIVE, SUCCEEDED, ABORTED, CANCELED = 'active', 'succeeded', 'aborted', 'canceled'


@dataclass
class Nav2Status:
    state: str = ACTIVE
    distance_remaining: float = float('nan')
    error_code: int = 0
    error_msg: str = ''


@dataclass
class Outcome:
    ok: bool
    code: str
    msg: str


class Nav2Client:
    """What the adapter needs from a Nav2 NavigateToPose client."""

    def wait_for_server(self, timeout_s):
        raise NotImplementedError

    def send_goal(self, x, y, yaw, frame_id):
        """Send a base_footprint goal. Returns True when Nav2 accepted it."""
        raise NotImplementedError

    def status(self):
        raise NotImplementedError

    def cancel(self):
        raise NotImplementedError


def footprint_goal(arm_x, arm_y, arm_yaw, mount):
    return arm_to_footprint(arm_x, arm_y, arm_yaw, mount)


def run_nav2_goal(client, arm_goal, mount, frame_id, timeout_s, is_cancelled=lambda: False,
                  on_feedback=lambda st: None, server_timeout_s=5.0, poll_s=0.05,
                  clock=time.monotonic, sleep=time.sleep):
    x, y, yaw = footprint_goal(*arm_goal, mount)
    if not all(map(math.isfinite, (x, y, yaw))):
        return Outcome(False, 'INVALID_GOAL', 'goal is not finite')
    if not client.wait_for_server(server_timeout_s):
        return Outcome(False, 'NO_NAV2', f'Nav2 navigate_to_pose server not available after {server_timeout_s:g} s')
    if not client.send_goal(x, y, yaw, frame_id):
        return Outcome(False, 'INVALID_GOAL', 'Nav2 rejected the goal')
    t0 = clock()
    while True:
        if is_cancelled():
            client.cancel()
            return Outcome(False, 'CANCELED', 'canceled')
        st = client.status()
        if st.state == SUCCEEDED:
            return Outcome(True, 'NONE', 'arrived (Nav2)')
        if st.state == CANCELED:
            return Outcome(False, 'CANCELED', 'Nav2 canceled the goal')
        if st.state == ABORTED:
            return Outcome(False, 'UNKNOWN', f'Nav2 aborted (error {st.error_code}): {st.error_msg}'.rstrip(': '))
        on_feedback(st)
        if clock() - t0 > timeout_s:
            client.cancel()
            return Outcome(False, 'TIMEOUT', f'Nav2 not there after {timeout_s:.0f} s')
        sleep(poll_s)


class RosNav2Client(Nav2Client):
    """rclpy client for nav2_msgs/NavigateToPose. Raises ImportError at construction if Nav2 is not installed."""

    def __init__(self, node, action_name='navigate_to_pose'):
        from nav2_msgs.action import NavigateToPose      # lazy: optional dependency
        from rclpy.action import ActionClient
        self._node, self._type = node, NavigateToPose
        self._client = ActionClient(node, NavigateToPose, action_name)
        self._handle = None
        self._state = Nav2Status()
        self._distance = float('nan')

    def wait_for_server(self, timeout_s):
        return self._client.wait_for_server(timeout_sec=timeout_s)

    def send_goal(self, x, y, yaw, frame_id):
        g = self._type.Goal()
        g.pose.header.frame_id = frame_id
        g.pose.header.stamp = self._node.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = x, y
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self._state = Nav2Status()
        fut = self._client.send_goal_async(g, feedback_callback=self._on_feedback)
        while not fut.done():            # the node is spun by another executor thread
            time.sleep(0.01)
        self._handle = fut.result()
        if not self._handle.accepted:
            return False
        self._handle.get_result_async().add_done_callback(self._on_result)
        return True

    def _on_feedback(self, fb):
        self._distance = float(fb.feedback.distance_remaining)

    def _on_result(self, fut):
        from action_msgs.msg import GoalStatus
        r = fut.result()
        s = {GoalStatus.STATUS_SUCCEEDED: SUCCEEDED, GoalStatus.STATUS_CANCELED: CANCELED}.get(r.status, ABORTED)
        self._state = Nav2Status(s, 0.0 if s == SUCCEEDED else self._distance,
                                 int(getattr(r.result, 'error_code', 0)), str(getattr(r.result, 'error_msg', '')))

    def status(self):
        if self._state.state == ACTIVE:
            return Nav2Status(ACTIVE, self._distance)
        return self._state

    def cancel(self):
        if self._handle is not None:
            self._handle.cancel_goal_async()
