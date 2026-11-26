"""Tests for wall_reach_report.py (numpy only, no ROS, no docker).

    python -m pytest docker/tools/tests/test_wall_reach_report.py
"""
import json
import pathlib
import sys

import numpy as np
import pytest

TOOLS = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))

import wall_reach_report as w  # noqa: E402


def test_ik_roundtrip_finds_the_generating_configuration():
    rng = np.random.default_rng(1)
    q = rng.uniform(-3.0, 3.0, (200, 6))
    q[:, 2] = rng.uniform(-3.0, 3.0, 200)   # elbow stays inside its +-pi
    T = w.fk(q)
    sols, ok = w.ik(T)
    for i in range(len(q)):
        errs = [np.abs(w.fk(sols[i, k]) - T[i]).max() for k in range(8) if ok[i, k]]
        assert errs and min(errs) < 1e-6
        assert all(e < 1e-6 for e in errs)   # every branch it returns is a true solution


def test_runs_and_group_strips():
    v = np.arange(6) * 0.1
    got = w.runs(np.array([1, 1, 0, 1, 0, 1], bool), v)
    assert [x for t in got for x in t] == pytest.approx([0.0, 0.1, 0.3, 0.3, 0.5, 0.5])
    ys = [i * 0.16 for i in range(19)]
    groups = w.group_strips(ys, 0.5)
    assert sum(len(g) for g in groups) == 19
    assert all(g[-1] - g[0] <= 0.5 + 1e-9 for g in groups)
    assert len(groups) == 5   # 4 strips (0.48 m) per stand, ceil(19 / 4)


def test_wall_z_mask_uses_the_lift_stroke():
    z_rel = np.round(np.arange(-1.0, 1.0001, 0.1), 6)
    rel = (z_rel >= -0.2) & (z_rel <= 0.2)
    zw = np.round(np.arange(0.0, 4.0001, 0.1), 6)
    m = w.wall_z_mask(rel, z_rel, zw, z0=1.0, lift_max=1.0)
    lo, hi = zw[m].min(), zw[m].max()
    assert lo == pytest.approx(0.8, abs=0.051) and hi == pytest.approx(2.2, abs=0.051)
    m0 = w.wall_z_mask(rel, z_rel, zw, z0=1.0, lift_max=0.0)
    assert zw[m0].min() == pytest.approx(0.8, abs=0.051) and zw[m0].max() == pytest.approx(1.2, abs=0.051)


def test_reach_straight_ahead_yes_far_away_no():
    y = np.array([0.0])
    z = np.array([0.0, 0.3])
    g_near = w.reachable_grid(0.6, y, z, link_x_max=0.6, roll_step_deg=45.0)
    assert g_near.all()
    g_far = w.reachable_grid(2.0, y, z, link_x_max=2.0, roll_step_deg=45.0)
    assert not g_far.any()


def test_elbow_must_stay_in_front_of_the_wall():
    y = np.array([0.0])
    z = np.array([0.0])
    # a wall so close that no wrist/elbow origin can sit in front of it
    assert not w.reachable_grid(0.6, y, z, link_x_max=-1.0, roll_step_deg=45.0).any()


def test_main_writes_json_and_reports_full_reach(tmp_path, capsys):
    out = tmp_path / 'r.json'
    rc = w.main(['--strips', '3', '--standoff', '0.7', '--stand-y', '0.16', '--max-lateral', '0.4',
                 '--roll-step-deg', '45', '--z-step', '0.05', '--y-step', '0.05', '--spans', '0.5',
                 '--json', str(out)])
    assert rc == 0
    data = json.loads(out.read_text())
    e = data['standoffs']['0.70']
    assert e['full_height_half_width_m'] >= 0.35
    assert len(data['per_strip']['strips']) == 3
    assert all(not s['z_missing'] for s in data['per_strip']['strips'])
    assert 'analytic' in capsys.readouterr().out


def test_unreachable_when_the_lift_cannot_help(tmp_path):
    out = tmp_path / 'r.json'
    # arm base 5 m up with no lift stroke: nothing at 0.5..1.9 m is reachable
    w.main(['--strips', '1', '--standoff', '0.7', '--arm-base-z0', '5', '--lift-max', '0', '--max-lateral', '0.2',
            '--roll-step-deg', '45', '--z-step', '0.05', '--y-step', '0.05', '--spans', '0.2',
            '--json', str(out)])
    assert json.loads(out.read_text())['standoffs']['0.70']['full_height_half_width_m'] is None
