"""Ray fan helper of the simulated lidar (sim_mujoco.lidar)."""
import math

import pytest

from sim_mujoco.lidar import LIDAR_DEFAULTS, lidar_numeric, range_to_wall, ray_angles, ray_count


def test_default_fan_is_270_degrees_in_half_degree_steps():
    d = LIDAR_DEFAULTS
    assert ray_count(d['angle_min'], d['angle_max'], d['angle_increment']) == 541
    a = ray_angles(d['angle_min'], d['angle_max'], d['angle_increment'])
    assert a[0] == pytest.approx(math.radians(-135))
    assert a[-1] == pytest.approx(math.radians(135))
    assert a[270] == pytest.approx(0.0, abs=1e-12)


def test_count_survives_rounded_angles():
    # 7 digit angles as in a hand-written MJCF: 89.9999971 steps still means 91 rays
    assert ray_count(-0.78539816, 0.78539816, 0.017453293) == 91
    assert ray_count(0.0, 0.0, 0.1) == 1
    assert ray_count(0.0, 0.25, 0.1) == 3  # a partial last step is not a ray


def test_bad_fans_are_refused():
    with pytest.raises(ValueError):
        ray_count(0.0, 1.0, 0.0)
    with pytest.raises(ValueError):
        ray_count(1.0, 0.0, 0.1)


def test_range_to_wall():
    assert range_to_wall(0.0, 2.0) == pytest.approx(2.0)
    assert range_to_wall(math.radians(60), 2.0) == pytest.approx(4.0)
    assert math.isinf(range_to_wall(math.radians(100), 2.0))
    assert math.isinf(range_to_wall(0.0, -1.0))


def test_numeric_has_the_nine_values_the_plugin_reads():
    n = lidar_numeric({})
    assert len(n) == 9
    assert n[3:9] == [0.05, 10.0, 10.0, 0.0, 0.0, 12345.0]
    assert lidar_numeric(dict(noise_std=0.02, dropout=0.1, seed=3))[6:9] == [0.02, 0.1, 3.0]


@pytest.mark.parametrize('bad', [dict(range_max=0.01), dict(rate_hz=0.0), dict(noise_std=-1.0), dict(dropout=1.5),
                                 dict(angle_increment=1e-6)])
def test_numeric_refuses_bad_values(bad):
    with pytest.raises(ValueError):
        lidar_numeric(bad)
