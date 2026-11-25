import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sim_overlay import mujoco_overlay as mo  # noqa: E402

PRIMS = [
    {'kind': 'polyline', 'points': [[0, 0, 0], [1, 0, 0], [1, 1, 0]], 'color': [1, 0, 0, 1], 'width': 0.02},
    {'kind': 'arrow', 'start': [0, 0, 0], 'end': [1, 0, 0], 'color': [0, 1, 0, 1], 'width': 0.05},
    {'kind': 'box', 'center': [0, 0, 0], 'size': [1, 1, 1], 'quat': [0, 0, 0, 1], 'color': [0, 0, 1, 1]},
    {'kind': 'label', 'position': [1, 1, 1], 'text': 'S1', 'height': 0.15, 'color': [1, 1, 1, 1]},
    {'kind': 'sphere', 'center': [0, 1, 0], 'radius': 0.05, 'color': [1, 1, 0, 1]},
]


def test_feed_roundtrip_atomic(tmp_path):
    p = tmp_path / 'sub' / 'o.json'
    mo.save_overlay(str(p), PRIMS)
    assert mo.load_overlay(str(p)) == PRIMS
    assert [f.name for f in p.parent.iterdir()] == ['o.json']   # no temp file left
    assert json.loads(p.read_text())['version'] == 1


def test_load_missing_or_corrupt(tmp_path):
    assert mo.load_overlay(str(tmp_path / 'none.json')) == []
    (tmp_path / 'bad.json').write_text('{"primitives": [1,')
    assert mo.load_overlay(str(tmp_path / 'bad.json')) == []


class FakeGeom:
    label = ''


class FakeScn:
    def __init__(self, n):
        self.maxgeom, self.ngeom = n, 5
        self.geoms = [FakeGeom() for _ in range(n)]


class FakeMj:
    class mjtGeom:
        mjGEOM_CAPSULE, mjGEOM_SPHERE = 3, 2
    calls = []

    @classmethod
    def mjv_initGeom(cls, g, t, *a):
        cls.calls.append(('init', t))

    @classmethod
    def mjv_connector(cls, g, t, w, a, b):
        cls.calls.append(('conn', w))


def test_cap_respected_with_fakes():
    pytest.importorskip('numpy')
    scn = FakeScn(4)
    dropped = mo.draw(scn, PRIMS, mujoco=FakeMj)
    assert scn.ngeom <= 4 and dropped >= 1


def test_real_mujoco():
    mujoco = pytest.importorskip('mujoco')
    m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom size="1" type="sphere"/></worldbody></mujoco>')
    scn = mujoco.MjvScene(m, maxgeom=100)
    assert mo.draw(scn, PRIMS) == 0
    # polyline 2 + arrow 4 + box 15 capsules, 1 label sphere, 1 sphere
    assert scn.ngeom == 2 + 4 + 15 + 1 + 1
    assert any(g.label == 'S1' for g in scn.geoms[:scn.ngeom])
    assert scn.geoms[0].type == mujoco.mjtGeom.mjGEOM_CAPSULE
    small = mujoco.MjvScene(m, maxgeom=10)
    dropped = mo.draw(small, PRIMS)
    assert small.ngeom <= 10 and dropped >= 1
    assert mo.draw(scn, []) == 0 and scn.ngeom == 0
