"""Every profile is valid YAML with a Visualization Manager, a fixed frame and at least one display; every display
topic is either a bare name or absolute (RViz keeps what is written)."""
import glob
import os

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PROFILES = sorted(glob.glob(os.path.join(HERE, '..', 'profiles', '*.rviz')))


def test_there_are_profiles():
    assert PROFILES


@pytest.mark.parametrize('path', PROFILES, ids=[os.path.basename(p) for p in PROFILES])
def test_profile_is_wellformed(path):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    vm = cfg['Visualization Manager']
    assert vm['Global Options']['Fixed Frame']
    displays = vm['Displays']
    assert any(d.get('Enabled', True) for d in displays), 'no enabled display'
    for d in displays:
        assert d.get('Class'), d
        assert d.get('Name'), d


def _sim_overview():
    with open(os.path.join(HERE, '..', 'profiles', 'sim_overview.rviz')) as f:
        return yaml.safe_load(f)


def test_sim_overview_topics_frame_and_window():
    cfg = _sim_overview()
    vm = cfg['Visualization Manager']
    assert vm['Global Options']['Fixed Frame'] == 'odom'
    assert cfg['Window Geometry']['Width'] <= 1200 and cfg['Window Geometry']['Height'] <= 800   # small window, cheap to draw
    topics = {}
    for d in vm['Displays']:
        t = d.get('Topic')
        topics[d['Name']] = (t.get('Value') if isinstance(t, dict) else t) or d.get('Trajectory Topic') \
            or d.get('Planning Scene Topic')
        if d.get('Enabled', True) and 'Topic' in d and isinstance(t, dict):
            assert t['Durability Policy'] in ('Volatile', 'Transient Local')
    enabled = {d['Name']: d for d in vm['Displays'] if d.get('Enabled', True)}
    want = ['/mp_viz/wall', '/mp_viz/plan_columns', '/mp_viz/stands',
            '/mp_viz/coverage_paint', '/mp_viz/transitions', '/mp_viz/stand_columns', '/mp_viz/tcp_executed',
            '/mp_viz/mission_text']
    on_topics = {topics[n] for n in enabled}
    for w in want:
        assert w in on_topics, w
    # displays that can block RViz at its splash screen are off by default; the tool trace is the secondary trail
    off = {d['Name'] for d in vm['Displays'] if not d.get('Enabled', True)}
    assert not any(d['Class'].startswith('moveit_rviz_plugin/') for d in enabled.values())
    assert 'Tool trace (coloured by force)' in off
    green = next(d for d in vm['Displays'] if d['Name'].startswith('TCP executed'))
    assert green['Class'] == 'rviz_default_plugins/Marker'
    assert any(d['Class'] == 'rviz_default_plugins/RobotModel' for d in enabled.values())
    assert any(d['Class'] == 'rviz_default_plugins/TF' for d in enabled.values())
    # latched publishers need a Transient Local subscription
    latched = {'/mp_viz/wall', '/mp_viz/plan_columns', '/mp_viz/stands', '/mp_viz/coverage_paint',
               '/mp_viz/stand_columns', '/robot_description'}
    for d in vm['Displays']:
        t = d.get('Topic')
        if isinstance(t, dict) and t.get('Value') in latched or d.get('Description Topic', {}).get('Value') in latched:
            t = t if isinstance(t, dict) else d['Description Topic']
            assert t['Durability Policy'] == 'Transient Local', d['Name']


def test_sim_launches_default_to_sim_overview():
    root = os.path.join(HERE, '..', '..', '..', 'simulation')
    for rel in ('sim_gazebo/launch/sim.launch.py', 'sim_mujoco/launch/sim_mujoco.launch.py'):
        with open(os.path.join(root, rel)) as f:
            assert "'rviz_profile', default_value='sim_overview'" in f.read(), rel


def test_sim_overview_draws_exactly_one_robot_and_is_light():
    """Every RobotModel / PlanningScene robot / Trajectory display draws a robot copy: only one may be enabled."""
    cfg = _sim_overview()
    vm = cfg['Visualization Manager']
    robots = [d for d in vm['Displays'] if d.get('Enabled', True) and (
        d['Class'] in ('rviz_default_plugins/RobotModel', 'moveit_rviz_plugin/Trajectory')
        or (d['Class'] == 'moveit_rviz_plugin/PlanningScene'
            and (d.get('Scene Robot') or {}).get('Show Robot Visual', True)))]
    assert [d['Class'] for d in robots] == ['rviz_default_plugins/RobotModel']
    assert robots[0].get('Collision Enabled') is False
    assert vm['Global Options']['Frame Rate'] <= 15
    tf = next(d for d in vm['Displays'] if d['Class'] == 'rviz_default_plugins/TF')
    assert not tf.get('Show Axes', True) and not tf.get('Show Names', False)
    assert all(not d.get('Enabled', True) for d in vm['Displays'] if 'tool_trace' in str(d.get('Topic')))
