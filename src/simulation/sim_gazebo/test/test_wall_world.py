"""Wall world generator tests (no Gazebo, no ROS install needed)."""
import json
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

from sim_gazebo import wall_world as ww  # noqa: E402

SMOKE = os.path.join(HERE, 'data', 'smoke_wall_scene.json')
OBJECTS = os.path.join(HERE, 'data', 'scene_model_objects.yaml')
REAL = os.path.join(HERE, '..', '..', '..', 'perception', 'coverage_planner', 'test', 'data',
                    'wall_scene.json')


def _models(sdf):
    root = ET.fromstring(sdf)
    return {m.get('name'): m for m in root.iter('model')}


def _min_corner_z(shape):
    half = np.array(shape.dims) / 2
    signs = [np.array(s) * 2 - 1 for s in np.ndindex(2, 2, 2)]
    return min((shape.position + shape.rotation @ (half * s))[2] for s in signs)


def test_smoke_scene_unaligned_geometry():
    shapes = ww.load_shapes(SMOKE, align=False)
    wall = shapes[0]
    assert wall.role == 'wall' and len(shapes) == 2
    assert wall.dims == pytest.approx((3.0, 2.0, ww.WALL_THICKNESS))
    # front face on x = 2, box behind it
    assert wall.position == pytest.approx([2.0 + ww.WALL_THICKNESS / 2, 0.0, 1.0])
    ob = shapes[1]
    assert ob.dims == pytest.approx((0.3, 0.3, 0.05))
    assert ob.position == pytest.approx([2.0 - 0.025, 0.75, 2.0 - 0.65])


def test_align_puts_wall_in_front_of_robot_on_the_floor():
    shapes = ww.load_shapes(SMOKE, align=True, standoff=0.8)
    wall = shapes[0]
    assert wall.rotation[:, 2] == pytest.approx([-1, 0, 0], abs=1e-9)
    assert wall.position[0] - ww.WALL_THICKNESS / 2 == pytest.approx(0.8)
    assert wall.position[1] == pytest.approx(0.0)
    assert _min_corner_z(wall) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.skipif(not os.path.isfile(REAL), reason='coverage_planner test data not present')
def test_real_wall_scene_aligns_and_keeps_obstacles():
    with open(REAL) as f:
        scene = json.load(f)
    expect = sum(1 for o in scene['obstacles'] if o['parent_wall'] == 0)
    shapes = ww.load_shapes(REAL, wall_id=0, align=True, standoff=1.0)
    assert len(shapes) == 1 + expect
    wall = shapes[0]
    n = wall.rotation[:, 2]
    assert n[0] < -0.99 and abs(n[1]) < 1e-6          # horizontal part faces the robot
    assert _min_corner_z(wall) == pytest.approx(0.0, abs=1e-9)
    for s in shapes[1:]:                              # obstacles stand proud of the wall face
        assert s.position[0] < wall.position[0]


def test_unknown_wall_id_raises():
    with pytest.raises(ValueError, match='no wall with id 7'):
        ww.load_shapes(SMOKE, wall_id=7)


def test_crop_drops_outside_obstacle():
    shapes = ww.load_shapes(SMOKE, align=False, crop_u=(-1.0, 0.0))
    assert [s.name for s in shapes] == ['wall_0']
    assert shapes[0].dims[0] == pytest.approx(1.0)


def test_sdf_is_wellformed_and_has_models(tmp_path):
    out = tmp_path / 'w.sdf'
    shapes = ww.generate_world(SMOKE, str(out), standoff=0.9)
    models = _models(out.read_text())
    assert {'ground', 'wall_0', 'obstacle_0'} <= set(models)
    assert len(shapes) == 2
    root = ET.parse(out).getroot()
    assert root.get('version') == '1.9'
    plugins = {p.get('filename') for p in root.iter('plugin')}
    assert 'gz-sim-physics-system' in plugins and 'gz-sim-forcetorque-system' in plugins
    wall_pose = [float(x) for x in models['wall_0'].find('pose').text.split()]
    assert wall_pose[0] == pytest.approx(0.95)
    assert models['wall_0'].find('.//box/size').text.split() == ['3.000000', '2.000000', '0.100000']


def test_scene_model_yaml(tmp_path):
    out = tmp_path / 'o.sdf'
    shapes = ww.generate_world(OBJECTS, str(out))
    assert [(s.name, s.kind, s.role) for s in shapes] == [
        ('panel', 'box', 'wall'), ('post', 'cylinder', 'obstacle')]
    assert {'panel', 'post'} <= set(_models(out.read_text()))


def test_scene_model_rejects_unsupported_type():
    with pytest.raises(ValueError, match='not supported'):
        ww.scene_model_shapes({'objects': [{'id': 'm', 'type': 'mesh'}]})


def test_quaternion_matrix_roundtrip():
    rng = np.random.default_rng(1)
    for _ in range(20):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        q2 = np.array(ww._quat_from_matrix(ww._matrix_from_quat(q)))
        assert abs(abs(np.dot(q, q2)) - 1.0) < 1e-9
