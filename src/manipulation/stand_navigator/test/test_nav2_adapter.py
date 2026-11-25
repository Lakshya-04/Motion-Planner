import math
import sys

import pytest

from stand_navigator import nav2_adapter as na
from stand_navigator.core import arm_to_footprint


class FakeClient(na.Nav2Client):
    def __init__(self, script, server=True, accept=True):
        self.script, self.server, self.accept = list(script), server, accept
        self.sent, self.cancelled = None, False

    def wait_for_server(self, timeout_s):
        return self.server

    def send_goal(self, x, y, yaw, frame_id):
        self.sent = (x, y, yaw, frame_id)
        return self.accept

    def status(self):
        return self.script.pop(0) if len(self.script) > 1 else self.script[0]

    def cancel(self):
        self.cancelled = True


def run(client, **kw):
    t = [0.0]

    def clock():
        return t[0]

    def sleep(s):
        t[0] += s
    return na.run_nav2_goal(client, kw.pop('goal', (2.0, 1.0, 0.5)), kw.pop('mount', (0.3, 0.1)), 'odom',
                            kw.pop('timeout_s', 10.0), clock=clock, sleep=sleep, **kw)


def test_goal_is_converted_to_footprint():
    c = FakeClient([na.Nav2Status(na.SUCCEEDED)])
    out = run(c)
    ex, ey, eyaw = arm_to_footprint(2.0, 1.0, 0.5, (0.3, 0.1))
    assert out.ok and out.code == 'NONE'
    assert c.sent == (pytest.approx(ex), pytest.approx(ey), pytest.approx(eyaw), 'odom')
    assert (ex, ey) != (2.0, 1.0)


def test_feedback_then_success():
    seen = []
    c = FakeClient([na.Nav2Status(distance_remaining=1.0), na.Nav2Status(distance_remaining=0.5),
                    na.Nav2Status(na.SUCCEEDED)])
    out = run(c, on_feedback=lambda s: seen.append(s.distance_remaining))
    assert out.ok and seen == [1.0, 0.5]


def test_aborted_maps_to_unknown_with_nav2_message():
    out = run(FakeClient([na.Nav2Status(na.ABORTED, error_code=801, error_msg='no path')]))
    assert not out.ok and out.code == 'UNKNOWN' and '801' in out.msg and 'no path' in out.msg


def test_timeout_cancels():
    c = FakeClient([na.Nav2Status()])
    out = run(c, timeout_s=1.0)
    assert out.code == 'TIMEOUT' and c.cancelled


def test_cancel_request_forwarded():
    c = FakeClient([na.Nav2Status()])
    flag = iter([False, False, True])
    out = run(c, is_cancelled=lambda: next(flag))
    assert out.code == 'CANCELED' and c.cancelled


def test_no_server_rejected_and_non_finite_goal():
    assert run(FakeClient([], server=False)).code == 'NO_NAV2'
    assert run(FakeClient([na.Nav2Status(na.SUCCEEDED)], accept=False)).code == 'INVALID_GOAL'
    assert run(FakeClient([]), goal=(float('nan'), 0, 0)).code == 'INVALID_GOAL'


def test_import_does_not_need_nav2():
    # the module imports nav2_msgs only inside RosNav2Client.__init__
    assert 'nav2_msgs' not in open(na.__file__).read().split('class RosNav2Client')[0].replace(
        'nav2_msgs/NavigateToPose', '').replace('`nav2_msgs`', '').replace('nav2_msgs is', '')
