"""OverlayHook with a fake mujoco/scene (no mujoco, no ROS)."""
import json
import os

from sim_mujoco.viewer_overlay import ENV_VAR, OverlayHook


def _hook(path, calls):
    return OverlayHook(str(path), 'fake_mujoco',
                       draw=lambda scn, prims, mj: calls.append((scn, prims, mj)),
                       load=lambda p: json.load(open(p))['primitives'])


def test_unset_or_empty_env_is_off():
    assert OverlayHook.from_env('mj', environ={}) is None
    assert OverlayHook.from_env('mj', environ={ENV_VAR: ''}) is None


def test_set_env_builds_hook(tmp_path):
    h = OverlayHook.from_env('mj', environ={ENV_VAR: str(tmp_path / 'o.json')})
    assert h is not None and h.path.endswith('o.json')


def test_draws_only_when_mtime_changes(tmp_path):
    p = tmp_path / 'o.json'
    calls = []
    h = _hook(p, calls)
    prim = [{'kind': 'sphere', 'center': [0, 0, 0], 'radius': 0.1}]
    p.write_text(json.dumps({'version': 1, 'primitives': prim}))
    assert h.update('scn') and calls == [('scn', prim, 'fake_mujoco')]
    assert not h.update('scn') and len(calls) == 1          # unchanged
    p.write_text(json.dumps({'version': 1, 'primitives': []}))
    os.utime(p, ns=(1, 1))                                   # force a different mtime
    assert h.update('scn') and len(calls) == 2


def test_missing_file_then_appears(tmp_path):
    p = tmp_path / 'o.json'
    calls = []
    h = _hook(p, calls)
    assert not h.update('scn')                               # None == initial last_mtime: nothing drawn
    p.write_text(json.dumps({'version': 1, 'primitives': []}))
    assert h.update('scn') and len(calls) == 1


def test_draw_error_does_not_raise(tmp_path):
    p = tmp_path / 'o.json'
    p.write_text('{"version": 1, "primitives": []}')

    def boom(*a):
        raise RuntimeError('x')
    h = OverlayHook(str(p), 'mj', draw=boom, load=lambda q: [])
    assert h.update('scn') is False


def test_real_feed_format_roundtrip(tmp_path):
    """The file overlay_dump.py writes (save_overlay) loads with the real load_overlay."""
    import pytest
    pytest.importorskip('numpy')
    import sys
    sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parents[2] / 'common' / 'sim_overlay'))
    from sim_overlay.mujoco_overlay import load_overlay, save_overlay
    p = tmp_path / 'o.json'
    prims = [{'kind': 'tiles', 'quads': [[[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]]], 'color': [0, 1, 0, 0.5]}]
    save_overlay(str(p), prims)
    assert json.load(open(p))['version'] == 1
    assert load_overlay(str(p)) == prims
