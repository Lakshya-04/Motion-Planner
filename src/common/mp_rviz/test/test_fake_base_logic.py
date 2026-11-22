"""Host tests of the body-twist integration of mp_fake_base.py (no rclpy)."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
from mp_fake_base_logic import integrate  # noqa: E402


def test_straight_line_along_heading():
    x, y, yaw = integrate(0, 0, math.pi / 2, 0.1, 0, 0, 1.0)
    assert (x, y) == pytest.approx((0.0, 0.1), abs=1e-12)


def test_lateral_in_body_frame():
    x, y, _ = integrate(1, 2, 0, 0, 0.2, 0, 0.5)
    assert (x, y) == pytest.approx((1.0, 2.1))


def test_turn_in_place():
    assert integrate(0, 0, 0, 0, 0, 0.3, 2.0) == pytest.approx((0, 0, 0.6))
