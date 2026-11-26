import math

import numpy as np
import pytest

from base_localisation.scan_match import body_twist, match, scan_to_points, transform


def room_scan(pose, n=541, half=4.0):
    """Ranges of a 270 degree sweep from `pose` (x, y, yaw) in an 8 x 8 room with a block in it."""
    x, y, yaw = pose
    angles = np.linspace(-2.35, 2.35, n)
    ranges = np.full(n, np.inf)
    walls = [((-half, -half), (half, -half)), ((half, -half), (half, half)), ((half, half), (-half, half)),
             ((-half, half), (-half, -half)),
             ((1.0, 1.0), (1.8, 1.0)), ((1.8, 1.0), (1.8, 1.4)), ((1.8, 1.4), (1.0, 1.4)), ((1.0, 1.4), (1.0, 1.0))]
    for k, a in enumerate(angles):
        d = np.array([math.cos(yaw + a), math.sin(yaw + a)])
        for (x1, y1), (x2, y2) in walls:
            e = np.array([x2 - x1, y2 - y1])
            m = np.array([[d[0], -e[0]], [d[1], -e[1]]])
            if abs(np.linalg.det(m)) < 1e-9:
                continue
            r, u = np.linalg.solve(m, [x1 - x, y1 - y])
            if r > 0.05 and 0 <= u <= 1:
                ranges[k] = min(ranges[k], r)
    return ranges, angles


def points(pose):
    r, a = room_scan(pose)
    return scan_to_points(r, a[0], a[1] - a[0], 0.05, 10.0)


def test_recovers_a_known_motion():
    ref, new = points((0.0, 0.0, 0.0)), points((0.04, 0.01, 0.03))
    res = match(ref, new)
    assert res.converged and res.inlier_frac > 0.8
    # the pose of the new frame in the reference frame
    assert res.x == pytest.approx(0.04, abs=2e-3)
    assert res.y == pytest.approx(0.01, abs=2e-3)
    assert res.yaw == pytest.approx(0.03, abs=2e-3)
    assert res.rms < 0.01


def test_zero_motion_is_zero():
    p = points((0.5, -0.5, 0.2))
    res = match(p, p.copy())
    assert abs(res.x) < 1e-3 and abs(res.y) < 1e-3 and abs(res.yaw) < 1e-3


def test_body_twist_straight_and_turning():
    res = match(points((0, 0, 0)), points((0.03, 0, 0.0)))
    vx, vy, wz = body_twist(res, 0.1)
    assert vx == pytest.approx(0.3, abs=0.03) and abs(vy) < 0.03 and abs(wz) < 0.03
    res = match(points((0, 0, 0)), points((0, 0, 0.05)))
    assert body_twist(res, 0.1)[2] == pytest.approx(0.5, abs=0.04)


def test_too_few_points_does_not_converge():
    res = match(np.zeros((3, 2)), np.zeros((3, 2)))
    assert not res.converged and res.inlier_frac == 0.0


def test_scan_to_points_drops_invalid_returns():
    pts = scan_to_points([1.0, float('inf'), float('nan'), 0.01, 20.0, 2.0], 0.0, 0.1, 0.05, 10.0)
    assert len(pts) == 2


def test_transform_round_trip():
    p = np.array([[1.0, 2.0], [-0.5, 0.3]])
    q = transform(p, 0.3, -0.2, 0.7)
    assert np.allclose(transform(q - np.array([0.3, -0.2]), 0, 0, -0.7), p, atol=1e-9)
