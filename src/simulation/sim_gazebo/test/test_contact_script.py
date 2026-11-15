"""contact_control_test.py cleanup and release helpers (no sim; needs rclpy and the script's imports)."""
import os
import sys

import numpy as np
import pytest

SCRIPTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts')


def _script():
    pytest.importorskip('rclpy')
    pytest.importorskip('xacro')
    pytest.importorskip('control_msgs')
    sys.path.insert(0, SCRIPTS)
    sys.path.insert(0, os.path.join(SCRIPTS, '..'))
    import contact_control_test
    return contact_control_test


class FakeNode:
    def __init__(self, fail_first=False):
        self.calls, self.fail_first = [], fail_first

    def set_jtac(self, **params):
        self.calls.append(('jtac', params))
        if self.fail_first and len(self.calls) == 1:
            raise RuntimeError('service call failed')
        return True

    def target(self, fz):
        self.calls.append(('target', fz))

    def set_max_offset(self, metres):
        self.calls.append(('offset', metres))
        return True


def test_restore_disengages_clears_target_and_puts_back_gains_and_clamp():
    ct = _script()
    node = FakeNode()
    saved = {'admittance__mass': [1.0] * 6}
    ct.restore(node, saved)
    assert node.calls == [('jtac', {'admittance__selected_axes': [False] * 6}), ('target', 0.0),
                          ('jtac', saved), ('offset', ct.DEFAULT_MAX_OFFSET_M)]


def test_restore_keeps_going_when_a_step_fails():
    ct = _script()
    node = FakeNode(fail_first=True)
    ct.restore(node, None)
    assert ('target', 0.0) in node.calls and ('offset', ct.DEFAULT_MAX_OFFSET_M) in node.calls


def test_last_high_time():
    ct = _script()
    log = [(0.5 * i, np.array([0, 0, -8.0 if i < 4 else 0.0, 0, 0, 0])) for i in range(10)]
    assert ct.last_high_time(log, 0.0, 5.0) == pytest.approx(1.5)
    assert ct.last_high_time(log, 2.5, 5.0) is None
    assert ct.last_high_time([], 0.0, 1.0) is None
