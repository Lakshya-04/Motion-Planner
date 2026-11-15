"""Tests for the glue in robot_assets.py (no docker, no GPU, no ROS).

    python -m pytest docker/tools/tests/test_robot_assets.py
"""
import json
import pathlib
import sys
import types

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import robot_assets as ra  # noqa: E402


def test_pinned_fixed_lift_uses_the_height_the_full_model_renders_at():
    """The full model defaults a fixed lift to 0.4 m; the arm-only model must match it."""
    profile = {'robot': {}, 'lift': {'type': 'fixed'}, 'home': {}}
    assert float(ra.arm_only_args(profile)['lift_fixed_height']) == pytest.approx(0.4)


def test_fixed_lift_keeps_an_explicit_xacro_height():
    profile = {'robot': {'xacro_args': {'lift_fixed_height': 0.55}}, 'lift': {'type': 'fixed'}, 'home': {'lift_joint': 0.1}}
    assert float(ra.arm_only_args(profile)['lift_fixed_height']) == pytest.approx(0.55)


def test_moving_lift_is_pinned_at_its_home_height():
    profile = {'robot': {}, 'lift': {'type': 'telescopic'}, 'home': {'lift_joint': 0.3}}
    args = ra.arm_only_args(profile)
    assert args['lift'] == 'fixed' and float(args['lift_fixed_height']) == pytest.approx(0.3)


class FakeModel:
    dof = ['j1', 'j2']

    def __init__(self, urdf, srdf=None):
        FakeModel.srdf = srdf

    def colliding_pairs(self, q):
        return [('a', 'b')] if list(q) == [0.0, 0.0] else []  # home collides, anything else is free


def _check_setup(tmp_path, monkeypatch, with_srdf):
    (tmp_path / 'vamp').mkdir()
    (tmp_path / 'vamp' / 'spherized.urdf').write_text('<robot/>')
    if with_srdf:
        (tmp_path / 'vamp' / 'robot.srdf').write_text('<robot/>')
    monkeypatch.setattr(ra, 'load_module', lambda path, name: types.SimpleNamespace(Model=FakeModel))
    return {'joints': {'arm': ['j1', 'j2']}, 'home': {'j1': 0.0, 'j2': 0.0}}


def test_a_failed_self_collision_check_fails_the_stage(tmp_path, monkeypatch):
    profile = _check_setup(tmp_path, monkeypatch, with_srdf=True)
    with pytest.raises(SystemExit):
        ra.inner_check(profile, tmp_path)
    assert json.loads((tmp_path / 'check.json').read_text())['ok'] is False


def test_check_needs_the_vamp_ignore_list(tmp_path, monkeypatch):
    profile = _check_setup(tmp_path, monkeypatch, with_srdf=False)
    with pytest.raises(SystemExit, match='srdf'):
        ra.inner_check(profile, tmp_path)


def test_spheres_stage_drops_a_stale_ignore_list(tmp_path, monkeypatch):
    (tmp_path / 'vamp').mkdir()
    (tmp_path / 'vamp' / 'robot.srdf').write_text('<robot/>')
    monkeypatch.setattr(ra, 'run', lambda cmd, **kw: None)
    ra.host_spheres({'_path': 'p.yaml', 'frames': {'tcp': 'tcp'}}, tmp_path, {})
    assert not (tmp_path / 'vamp' / 'robot.srdf').exists()


def test_srdf_stage_passes_lift_stages(tmp_path, monkeypatch):
    import xacro
    seen = {}

    def fake(path, mappings):
        seen.update(mappings)
        return types.SimpleNamespace(toxml=lambda: '<robot/>')
    monkeypatch.setattr(xacro, 'process_file', fake)
    profile = {'robot': {}, 'lift': {'type': 'scissor', 'stages': 3, 'max_velocity_mps': 0.1},
               'joints': {'lift': None}}
    profile['robot'] = {'xacro_args': {'arm': 'ur10e'}}
    ra.inner_srdf(profile, tmp_path)
    assert seen['lift_stages'] == '3'
