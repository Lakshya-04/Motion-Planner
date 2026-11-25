"""Host test (no ROS, no sim): the two run_multi_stand_pass.sh scripts parse, document every env knob they read, read every
knob they document, share the parity knobs and print the one-line knob summary; the Gazebo wall shift handles a patch z."""
import json
import os
import re
import subprocess
import sys

import pytest

SIM = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'src', 'simulation')
SCRIPTS = {
    'gazebo': os.path.join(SIM, 'sim_gazebo', 'scripts', 'run_multi_stand_pass.sh'),
    'mujoco': os.path.join(SIM, 'sim_mujoco', 'scripts', 'run_multi_stand_pass.sh'),
}
VAR = re.compile(r'\b(?:MULTI_STAND|MANAGER_PASS)_[A-Z0-9_]+')
SHARED = ['MULTI_STAND_DOUBLE_PASS', 'MULTI_STAND_REACH_FRACTION', 'MULTI_STAND_ARM_REACH', 'MULTI_STAND_LIFT_STROKE',
          'MULTI_STAND_MIN_CLEARANCE', 'MULTI_STAND_PATCH_Z', 'MULTI_STAND_COVERAGE_EXTRA', 'MULTI_STAND_MAX_SPAN']
# `MULTI_STAND_SPEED / _SWATH / ...` style shorthand in a header line expands to MULTI_STAND_<X>
SHORT = re.compile(r'\s/\s_([A-Z0-9_]+)')


def split(path):
    header, body = [], []
    for line in open(path):
        (header if line.startswith('#') else body).append(line)
    return '\n'.join(header), '\n'.join(body)


def documented(header):
    names = set(VAR.findall(header))
    for line in header.splitlines():
        m = re.search(r'\b(MULTI_STAND)_[A-Z0-9_]+((?:\s*/\s*_[A-Z0-9_]+)+)', line)
        if m:
            names |= {'MULTI_STAND_' + s for s in SHORT.findall(' ' + m.group(0))}
            names |= {'MULTI_STAND_' + s.strip(' /_') for s in re.findall(r'/\s*_[A-Z0-9_]+', m.group(0))}
    return names


@pytest.mark.parametrize('name', SCRIPTS)
def test_bash_syntax(name):
    subprocess.run(['bash', '-n', SCRIPTS[name]], check=True)


@pytest.mark.parametrize('name', SCRIPTS)
def test_header_matches_variables_read(name):
    header, body = split(SCRIPTS[name])
    doc, read = documented(header), set(VAR.findall(body))
    assert read - doc == set(), f'{name}: read but not documented in the header: {sorted(read - doc)}'
    assert doc - read == set(), f'{name}: documented but never read: {sorted(doc - read)}'


@pytest.mark.parametrize('name', SCRIPTS)
def test_shared_knobs_and_summary(name):
    header, body = split(SCRIPTS[name])
    for var in SHARED:
        assert var in documented(header), f'{name}: {var} not documented'
        assert var in set(VAR.findall(body)), f'{name}: {var} not read'
    assert 'double_pass:=${MULTI_STAND_DOUBLE_PASS:-false}' in body
    assert 'stand_min_clearance_m:=${MULTI_STAND_MIN_CLEARANCE:-0.0}' in body
    for key in ('dock_ik_reach_fraction: ${MULTI_STAND_REACH_FRACTION:-0.0}', 'dock_ik_reach_m: ${MULTI_STAND_ARM_REACH:-1.3}',
                'dock_ik_lift_max_m: ${MULTI_STAND_LIFT_STROKE:-1.5}'):
        assert key in body, f'{name}: manager.yaml lacks {key}'
    assert body.count('multi_stand knobs:') == 1
    assert body.count('summary_line\n') >= 2   # the cancel exit and the normal exit


def test_gazebo_patch_z_shift(tmp_path):
    data = os.path.join(SIM, 'sim_gazebo', 'test', 'data')
    script = os.path.join(SIM, 'sim_gazebo', 'scripts', 'shift_wall_scene.py')
    subprocess.run([sys.executable, script, data, str(tmp_path), '0', '0.3', '0.9', '0.6,2.0'], check=True)
    cov = json.load(open(tmp_path / 'wall_pass_coverage_scene.json'))['walls'][0]
    sim = json.load(open(tmp_path / 'smoke_wall_scene.json'))['walls'][0]
    assert cov['origin_xyz'][2] == pytest.approx(1.3)
    assert cov['v_range_m'] == pytest.approx([-0.7, 0.7])
    assert sim['v_range_m'] == pytest.approx([0.0, 2.3])   # the sim wall holds the patch with 0.3 m above
    zs = [float(line.split()[2]) for line in open(tmp_path / 'walls.pcd').read().split('DATA ascii\n')[1].splitlines()]
    assert min(zs) == pytest.approx(0.55) and max(zs) == pytest.approx(2.05)
