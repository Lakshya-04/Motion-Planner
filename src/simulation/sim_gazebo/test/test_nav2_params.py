"""Nav2 parameter set: parses, matches the base profile, and the launch file lists the servers it configures."""
import ast
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS = os.path.join(HERE, '..', 'config', 'nav2', 'nav2_omni_params.yaml')
LAUNCH = os.path.join(HERE, '..', 'launch', 'nav2.launch.py')


def _params():
    with open(PARAMS) as f:
        return yaml.safe_load(f)


def test_footprint_is_the_profile_rectangle():
    p = _params()
    for cm in ('local_costmap', 'global_costmap'):
        fp = ast.literal_eval(p[cm][cm]['ros__parameters']['footprint'])
        assert sorted(map(tuple, fp)) == sorted([(-0.45, -0.3), (0.45, -0.3), (0.45, 0.3), (-0.45, 0.3)])


def test_omni_controller_and_odom_frame():
    p = _params()
    mppi = p['controller_server']['ros__parameters']['FollowPath']
    assert mppi['motion_model'] == 'Omni'
    assert mppi['vy_max'] > 0
    for cm in ('local_costmap', 'global_costmap'):
        assert p[cm][cm]['ros__parameters']['global_frame'] == 'odom'


def test_local_costmap_reads_scan_and_depth():
    v = _params()['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']
    assert v['scan']['topic'] == '/scan'
    assert v['depth']['topic'] == '/depth_sensor/points'


def test_launch_servers_have_a_parameter_block():
    with open(LAUNCH) as f:
        tree = ast.parse(f.read())
    servers = next(ast.literal_eval(n.value) for n in tree.body
                   if isinstance(n, ast.Assign) and n.targets[0].id == 'SERVERS')
    assert set(servers) <= set(_params())
