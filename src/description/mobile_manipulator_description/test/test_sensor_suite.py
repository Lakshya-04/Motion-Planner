"""Resolution of a platform's sensor suite into the simulators' lidar_* / depth_sensor_* values."""
import math
import pathlib
import sys

import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from mobile_manipulator_description.sensor_suite import (  # noqa: E402
    DEFAULTS, SuiteError, resolve_sensor_args, suite_values)

RANGER = 'ur10e_ranger_mini_telescopic'


def test_no_platform_is_the_old_defaults():
    assert resolve_sensor_args({}) == DEFAULTS
    assert resolve_sensor_args({k: '' for k in DEFAULTS}) == DEFAULTS
    assert DEFAULTS['lidar_rate_hz'] == '10.0' and DEFAULTS['lidar_pose'] == '0.45 0 0.2 0 0 0'


def test_suite_fills_lidar_and_depth():
    v = resolve_sensor_args({}, RANGER, want_lidar=True, want_depth=True)
    assert v['lidar_rate_hz'] == '40' and v['depth_sensor_rate_hz'] == '30'
    assert (v['lidar_angle_min_deg'], v['lidar_angle_max_deg'], v['lidar_angle_increment_deg']) == ('-135', '135', '0.25')
    assert (v['lidar_range_min_m'], v['lidar_range_max_m'], v['lidar_noise_m']) == ('0.06', '10', '0.03')
    assert v['lidar_frame'] == 'lidar_link' and v['depth_sensor_frame'] == 'front_camera_link'


def test_mount_is_moved_from_base_link_to_base_footprint():
    # ranger_mini: ground clearance 0.04, lift mount 0.264, so the body centre is 0.152 m up
    v = suite_values(RANGER)
    assert [float(x) for x in v['lidar_pose'].split()] == pytest.approx([0.30, 0.0, 0.16 + 0.152, 0, 0, 0])
    assert [float(x) for x in v['depth_sensor_pose'].split()] == pytest.approx([0.33, 0.0, 0.22 + 0.152, 0, 0, 0])


def test_explicit_args_win_over_the_suite():
    v = resolve_sensor_args({'lidar_rate_hz': '12.5', 'lidar_pose': '1 2 3 0 0 0', 'lidar_noise_m': '0.0',
                             'depth_sensor_rate_hz': ''}, RANGER, want_lidar=True, want_depth=True)
    assert v['lidar_rate_hz'] == '12.5' and v['lidar_pose'] == '1 2 3 0 0 0' and v['lidar_noise_m'] == '0.0'
    assert v['lidar_range_max_m'] == '10' and v['depth_sensor_rate_hz'] == '30'


def test_suite_without_a_noise_figure_keeps_the_default_noise():
    v = resolve_sensor_args({}, 'ur10e_scout_mini_telescopic', want_lidar=True)
    assert v['lidar_noise_m'] == DEFAULTS['lidar_noise_m']
    assert v['lidar_rate_hz'] == '10'


def test_lidar_only_suite_keeps_depth_defaults_and_refuses_depth_when_asked():
    name = 'ur10e_scout_telescopic'
    v = resolve_sensor_args({}, name, want_lidar=True)
    assert v['depth_sensor_pose'] == DEFAULTS['depth_sensor_pose'] and v['lidar_rate_hz'] == '40'
    with pytest.raises(SuiteError, match='no rgbd'):
        resolve_sensor_args({}, name, want_depth=True)


def test_platform_without_sensors_refuses_a_lidar():
    with pytest.raises(SuiteError, match='no lidar2d'):
        resolve_sensor_args({}, 'ur10e_telescopic', want_lidar=True)
    assert resolve_sensor_args({}, 'ur10e_telescopic') == DEFAULTS


def test_base_name_picks_the_one_profile_that_has_sensors():
    assert suite_values('ranger_mini') == suite_values(RANGER)
    assert suite_values('husky') == suite_values('ur10e_husky_telescopic')


def test_unknown_platform_lists_the_choices():
    with pytest.raises(SuiteError, match='unknown platform nope.*' + RANGER):
        suite_values('nope')


def test_a_profile_path_works(tmp_path):
    data = yaml.safe_load((PKG / 'config' / 'profiles' / f'{RANGER}.yaml').read_text())
    data['sensors'][0]['spec']['rate_hz'] = 20.0
    p = tmp_path / 'mine.yaml'
    p.write_text(yaml.safe_dump(data))
    assert suite_values(str(p))['lidar_rate_hz'] == '20'


def test_missing_body_height_is_an_error(tmp_path):
    data = yaml.safe_load((PKG / 'config' / 'profiles' / f'{RANGER}.yaml').read_text())
    del data['base']['lift_mount_height_m']
    p = tmp_path / 'bad.yaml'
    p.write_text(yaml.safe_dump(data))
    with pytest.raises(SuiteError, match='lift_mount_height_m'):
        suite_values(str(p))


@pytest.mark.parametrize('path', sorted((PKG / 'config' / 'profiles').glob('*.yaml')), ids=lambda p: p.stem)
def test_every_profile_resolves_to_numbers(path):
    v = suite_values(str(path))
    for key, val in v.items():
        if key.endswith('_frame'):
            continue
        assert all(math.isfinite(float(x)) for x in val.split()), key
    if 'lidar_pose' in v:
        assert float(v['lidar_angle_min_deg']) < float(v['lidar_angle_max_deg'])
        assert float(v['lidar_angle_increment_deg']) > 0
