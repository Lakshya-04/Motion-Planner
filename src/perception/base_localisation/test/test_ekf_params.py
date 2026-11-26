import importlib.util
import os
import sys

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

from base_localisation import ekf_params  # noqa: E402

NAMES = ['x', 'y', 'z', 'roll', 'pitch', 'yaw', 'vx', 'vy', 'vz', 'vroll', 'vpitch', 'vyaw', 'ax', 'ay', 'az']


def fused(params, key):
    return {n for n, on in zip(NAMES, params[key]) if on}


@pytest.mark.parametrize('platform', ekf_params.PLATFORMS)
def test_every_config_is_a_valid_filter_setup(platform):
    p = ekf_params.build(platform, imu=True, scan_match=True)
    assert p['two_d_mode'] is True and p['world_frame'] == p['odom_frame'] == 'odom'
    assert p['base_link_frame'] == 'base_footprint'
    for key in ('odom0_config', 'imu0_config', 'odom1_config'):
        assert len(p[key]) == 15 and all(isinstance(v, bool) for v in p[key])
    cov = p['process_noise_covariance']
    assert len(cov) == 225 and all(cov[i * 15 + i] > 0 for i in range(15))
    assert not any(v for i, v in enumerate(cov) if i // 15 != i % 15)
    # velocities only: no pose is fused from the wheels, the scan matcher or the IMU
    for key in ('odom0_config', 'imu0_config', 'odom1_config'):
        assert not fused(p, key) & {'x', 'y', 'yaw', 'roll', 'pitch'}, key
    assert fused(p, 'imu0_config') == {'vyaw'}


def test_vy_only_on_the_omni_class():
    p = ekf_params.build('omni', scan_match=True)
    assert fused(p, 'odom0_config') == fused(p, 'odom1_config') == {'vx', 'vy', 'vyaw'}
    for platform in ('diff', 'ackermann'):
        p = ekf_params.build(platform, scan_match=True)
        assert fused(p, 'odom0_config') == fused(p, 'odom1_config') == {'vx', 'vyaw'}


def test_sources_that_are_off_are_dropped():
    p = ekf_params.build('omni', imu=False, scan_match=False)
    assert not [k for k in p if k.startswith(('imu0', 'odom1'))]
    assert p['odom0'] == '/odom'
    p = ekf_params.build('omni', imu=True, scan_match=False)
    assert 'imu0' in p and 'odom1' not in p
    p = ekf_params.build('omni', imu=False, scan_match=True)
    assert 'odom1' in p and 'imu0' not in p


def test_topics_tf_and_sim_time_are_passed_through():
    p = ekf_params.build('diff', odom_topic='/wheel/odom', imu_topic='/imu/data', scan_match=True,
                         scan_odom_topic='/sm', publish_tf=False, use_sim_time=True)
    assert (p['odom0'], p['imu0'], p['odom1']) == ('/wheel/odom', '/imu/data', '/sm')
    assert p['publish_tf'] is False and p['use_sim_time'] is True


def test_unknown_platform_and_bad_diag_fail_loudly(tmp_path):
    with pytest.raises(ValueError):
        ekf_params.build('hexapod')
    bad = tmp_path / 'bad.yaml'
    bad.write_text(yaml.safe_dump({'ekf_filter_node': {'ros__parameters': {'process_noise_diag': [1.0]}}}))
    with pytest.raises(ValueError):
        ekf_params.build('omni', path=str(bad))


def test_launch_file_builds_the_nodes():
    pytest.importorskip('launch_ros')
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    spec = importlib.util.spec_from_file_location('loc_launch', os.path.join(HERE, '..', 'launch',
                                                                           'localisation.launch.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    def nodes(**overrides):
        ctx = LaunchContext()
        for a in mod.generate_launch_description().entities:
            if isinstance(a, DeclareLaunchArgument):
                ctx.launch_configurations[a.name] = overrides.get(
                    a.name, ''.join(t.perform(ctx) for t in a.default_value))
        return mod._nodes(ctx)

    only = nodes()
    assert len(only) == 1 and only[0].node_executable == 'ekf_node'
    both = nodes(scan_match='true', platform='diff')
    assert [n.node_executable for n in both] == ['ekf_node', 'scan_match_node']
