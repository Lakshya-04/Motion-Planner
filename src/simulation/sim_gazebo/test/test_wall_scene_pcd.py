"""walls_scene.pcd (shift_wall_scene.sim_wall_scene_pcd): the whole sim wall for MoveIt's scene (no ROS needed)."""
import json
import os
import subprocess
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, '..', 'scripts', 'shift_wall_scene.py')
DATA = os.path.join(HERE, 'data')
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))

import shift_wall_scene as sws  # noqa: E402


def _run(tmp_path, dy, width, z=None):
    args = [sys.executable, '-I', SCRIPT, DATA, str(tmp_path), '0.0', str(dy), str(width)] + ([z] if z else [])
    subprocess.run(args, check=True)
    return tmp_path


def _cloud(path):
    rows = []
    with open(path) as f:
        n, data = None, False
        for line in f:
            if data and line.strip():
                rows.append([float(v) for v in line.split()])
            elif line.startswith('POINTS'):
                n = int(line.split()[1])
            data = data or line.startswith('DATA')
    assert n == len(rows)
    return np.array(rows)


def test_stock_wall_whole_face(tmp_path):
    _run(tmp_path, 0.0, sws.PATCH_WIDTH)
    c = _cloud(tmp_path / 'walls_scene.pcd')
    assert np.all(c[:, 0] == pytest.approx(0.98))          # the tool's contact plane
    assert c[:, 1].min() == pytest.approx(-1.5) and c[:, 1].max() == pytest.approx(1.5)   # u_range -1.5..1.5 (shift 0.15 + grow 0)
    assert c[:, 2].min() == pytest.approx(0.0) and c[:, 2].max() == pytest.approx(2.0)
    assert len(c) == 301 * 201


def test_spacing_is_octomap_resolution(tmp_path):
    _run(tmp_path, 0.0, sws.PATCH_WIDTH)
    c = _cloud(tmp_path / 'walls_scene.pcd')
    assert np.diff(np.unique(c[:, 1])) == pytest.approx(0.01, abs=1e-6)
    assert np.diff(np.unique(c[:, 2])) == pytest.approx(0.01, abs=1e-6)


def test_patch_cloud_untouched_and_inside(tmp_path):
    _run(tmp_path, 0.3, 0.9)
    patch, whole = _cloud(tmp_path / 'walls.pcd'), _cloud(tmp_path / 'walls_scene.pcd')
    assert len(patch) < len(whole)
    for k in (1, 2):
        assert whole[:, k].min() <= patch[:, k].min() and whole[:, k].max() >= patch[:, k].max()


def test_wide_shifted_wall(tmp_path):
    _run(tmp_path, 0.3, 0.9)
    c = _cloud(tmp_path / 'walls_scene.pcd')
    w = json.load(open(tmp_path / 'smoke_wall_scene.json'))['walls'][0]
    assert c[:, 1].min() == pytest.approx(w['origin_xyz'][1] + w['u_range_m'][0])
    assert c[:, 1].max() == pytest.approx(w['origin_xyz'][1] + w['u_range_m'][1])
    assert c[:, 1].min() < 0.3 - 0.25 + 1e-6 and c[:, 1].max() > 1.2   # covers the patch y 0.3..1.2 and its margin


def test_tall_wall_reaches_top(tmp_path):
    _run(tmp_path, 0.3, 0.9, '0.6,2.0')
    c = _cloud(tmp_path / 'walls_scene.pcd')
    assert c[:, 2].min() == pytest.approx(0.0) and c[:, 2].max() == pytest.approx(2.3)   # HI + 0.3
    patch = _cloud(tmp_path / 'walls.pcd')
    assert patch[:, 2].max() < 2.3                     # the patch cloud keeps only the contact patch


def test_empty_cloud_rejected(tmp_path):
    empty = tmp_path / 'e.pcd'
    empty.write_text('VERSION 0.7\nDATA ascii\n')
    with pytest.raises(SystemExit):
        sws.sim_wall_scene_pcd(os.path.join(DATA, 'smoke_wall_scene.json'), str(empty), str(tmp_path / 'o.pcd'))
