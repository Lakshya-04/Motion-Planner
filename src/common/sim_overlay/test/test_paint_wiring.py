"""The coverage paint is reachable from the viewers: RViz profiles carry the display, both multi-stand scripts the knob."""
import os
import subprocess

import pytest
import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
TOPIC = '/mp_viz/coverage_paint'


def _displays(ds):
    for d in ds:
        yield d
        yield from _displays(d.get('Displays', []))


@pytest.mark.parametrize('profile', ['layers', 'mission_overview'])
def test_rviz_profile_has_paint_display_off(profile):
    with open(os.path.join(ROOT, 'common', 'mp_rviz', 'profiles', profile + '.rviz')) as f:
        cfg = yaml.safe_load(f)
    found = [d for d in _displays(cfg['Visualization Manager']['Displays']) if d.get('Topic', {}).get('Value') == TOPIC]
    assert len(found) == 1 and found[0]['Enabled'] is False


@pytest.mark.parametrize('sim', ['mujoco', 'gazebo'])
def test_multi_stand_script_paint_knob(sim):
    path = os.path.join(ROOT, 'simulation', 'sim_' + sim, 'scripts', 'run_multi_stand_pass.sh')
    text = open(path).read()
    assert 'MULTI_STAND_PAINT:-0}" = 1' in text            # default off
    assert 'coverage_paint.launch.py' in text and 'source:=' + sim in text
    assert subprocess.run(['bash', '-n', path]).returncode == 0
