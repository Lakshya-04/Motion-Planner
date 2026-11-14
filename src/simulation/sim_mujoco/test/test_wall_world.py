"""Wall world generator tests (pure Python; compile checks need `mujoco`)."""
import json
import math
import pathlib
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from sim_mujoco.wall_world import (apply_offset, load_scene, quat_rotate, wall_world_geoms)

DATA = pathlib.Path(__file__).parent / 'data'
REPO_SCENE = None
for parent in pathlib.Path(__file__).resolve().parents:
    cand = parent / 'src' / 'perception' / 'coverage_planner' / 'test' / 'data' / 'wall_scene.json'
    if cand.is_file():
        REPO_SCENE = cand
        break
SMOKE = DATA / 'smoke_wall_scene.json'


def test_smoke_wall_plane_and_orientation():
    prims, skipped = load_scene(SMOKE)
    assert skipped == [] and len(prims) == 1
    wall = prims[0]
    # front face on x = 0.98: centre is 0.05 behind (thickness 0.1), half sizes (u, v, n)
    assert wall.pos == pytest.approx((1.03, 0.15, 1.15))
    assert wall.size == pytest.approx((0.4, 0.35, 0.05))
    # local z is the outward normal (-x): rotating (0, 0, 1) gives (-1, 0, 0)
    assert quat_rotate(wall.quat, (0, 0, 1)) == pytest.approx((-1, 0, 0), abs=1e-9)
    assert quat_rotate(wall.quat, (1, 0, 0)) == pytest.approx((0, 1, 0), abs=1e-9)


@pytest.mark.skipif(REPO_SCENE is None, reason='coverage_planner test data not in this tree')
def test_repo_wall_scene_has_every_wall_and_obstacle():
    scene = json.loads(REPO_SCENE.read_text())
    prims, _ = load_scene(REPO_SCENE)
    walls = [p for p in prims if p.name.startswith('wall_')]
    obstacles = [p for p in prims if p.name.startswith('obstacle_')]
    assert len(walls) == len(scene['walls']) and len(obstacles) == len(scene['obstacles'])
    for p, w in zip(walls, scene['walls']):
        n = np.array(w['normal_outward'])
        # the front face lies on the scanned plane: a, b, c, d of the plane equation
        face = np.array(p.pos) + quat_rotate(p.quat, (0, 0, p.size[2]))
        a, b, c, d = w['plane_abcd']
        assert a * face[0] + b * face[1] + c * face[2] + d == pytest.approx(0.0, abs=2e-3)
        assert quat_rotate(p.quat, (0, 0, 1)) == pytest.approx(tuple(n / np.linalg.norm(n)), abs=1e-6)
    ob = scene['obstacles'][0]
    assert obstacles[0].size == pytest.approx(tuple(e / 2 for e in ob['box_extents_uvn']))
    assert obstacles[0].pos == pytest.approx(tuple(ob['box_center_xyz']))
    only = load_scene(REPO_SCENE, wall_ids={0}, with_obstacles=False)[0]
    assert [p.name for p in only] == ['wall_0']


def test_offset_moves_and_turns_the_scene():
    prims, _ = load_scene(SMOKE)
    moved = apply_offset(prims, (1.0, 2.0, 3.0, math.pi / 2))
    # yaw 90 deg: (1.03, 0.15, 1.15) -> (-0.15, 1.03, 1.15), then translate
    assert moved[0].pos == pytest.approx((0.85, 3.03, 4.15))
    assert quat_rotate(moved[0].quat, (0, 0, 1)) == pytest.approx((0, -1, 0), abs=1e-9)


def test_native_scene_model_file(tmp_path):
    scene = tmp_path / 'scene.yaml'
    scene.write_text("""
frame_id: base_link
objects:
  - {id: panel, type: box, size: [0.1, 2.0, 2.0], role: contact_surface,
     pose: {position: [1.0, 0.0, 1.0], orientation_xyzw: [0, 0, 0, 1]}}
  - {id: ball, type: sphere, radius: 0.1, pose: {position: [0.5, 0.2, 0.3]}}
  - {id: post, type: cylinder, radius: 0.05, height: 1.0, pose: {position: [0.4, 0, 0.5]}}
  - {id: cloud, type: pointcloud, file: x.pcd}
""")
    geoms, prims, skipped = wall_world_geoms(scene)
    assert [p.name for p in prims] == ['panel', 'ball', 'post']
    assert skipped == ['cloud (pointcloud)']
    assert geoms[0].get('type') == 'box' and geoms[0].get('size') == '0.05 1 1'
    assert geoms[0].get('friction') is not None and geoms[1].get('friction') is None


def test_curobo_world(tmp_path):
    scene = tmp_path / 'world.yaml'
    scene.write_text("cuboid:\n  table: {dims: [1.0, 2.0, 0.1], pose: [0, 0, 0.5, 1, 0, 0, 0]}\n")
    prims, _ = load_scene(scene)
    assert prims[0].size == pytest.approx((0.5, 1.0, 0.05))


def test_bad_scene_is_rejected(tmp_path):
    bad = tmp_path / 'bad.json'
    bad.write_text('{"nothing": 1}')
    with pytest.raises(ValueError):
        load_scene(bad)


mujoco = pytest.importorskip('mujoco')


def test_world_compiles_and_blocks_a_falling_ball(tmp_path):
    geoms, _, _ = wall_world_geoms(SMOKE)
    root = ET.Element('mujoco')
    ET.SubElement(root, 'option', timestep='0.002')
    world = ET.SubElement(root, 'worldbody')
    for g in geoms:
        world.append(g)
    ball = ET.SubElement(world, 'body', pos='1.0 0.15 1.4')
    ET.SubElement(ball, 'freejoint')
    ET.SubElement(ball, 'geom', type='sphere', size='0.03', mass='0.1')
    path = tmp_path / 'w.xml'
    ET.ElementTree(root).write(path)
    m = mujoco.MjModel.from_xml_path(str(path))
    d = mujoco.MjData(m)
    # a ball thrown at the wall from the robot side stops at its front face (x = 0.98)
    d.qpos[:3] = (0.5, 0.15, 1.15)
    d.qvel[:3] = (2.0, 0, 0)
    m.opt.gravity[:] = 0
    reach = 0.0
    for _ in range(1500):
        mujoco.mj_step(m, d)
        reach = max(reach, d.qpos[0])
    assert 0.90 < reach < 0.98
