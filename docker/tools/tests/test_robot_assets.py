"""Tests for the glue in robot_assets.py (no docker, no GPU, no ROS).

    python -m pytest docker/tools/tests/test_robot_assets.py
"""
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
