"""Tests for the pure parts of the MuJoCo cross-check (no GPU, no ROS, no mesh files).

    python -m pytest docker/tools/tests/test_mujoco_crosscheck.py

The MuJoCo test is skipped when `mujoco` is not importable (it is in motion-planner:sim).
"""
import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import mujoco_model as mm  # noqa: E402

# Three-link serial arm: world -> a (fixed base) -> b (revolute z) -> c (revolute y) -> tip (fixed).
# `rigid` hangs off the base by a fixed joint (same rigid group as a).
URDF = """<?xml version="1.0"?>
<robot name="t">
  <link name="a"><inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link>
  <link name="rigid"/>
  <link name="b"><inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><origin xyz="0.25 0 0"/><geometry><box size="0.5 0.1 0.1"/></geometry></collision></link>
  <link name="c"><inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><origin xyz="0.25 0 0"/><geometry><box size="0.5 0.1 0.1"/></geometry></collision></link>
  <link name="tip"/>
  <joint name="j_rigid" type="fixed"><parent link="a"/><child link="rigid"/><origin xyz="0 0 0.1"/></joint>
  <joint name="j1" type="revolute"><parent link="a"/><child link="b"/><origin xyz="0 0 0.12" rpy="0 0 0.1"/>
    <axis xyz="0 0 1"/><limit lower="-1.5" upper="1.5" effort="1" velocity="1"/></joint>
  <joint name="j2" type="revolute"><parent link="b"/><child link="c"/><origin xyz="0.5 0 0"/>
    <axis xyz="0 1 0"/><limit lower="-3.2" upper="3.2" effort="1" velocity="1"/></joint>
  <joint name="j_tip" type="fixed"><parent link="c"/><child link="tip"/><origin xyz="0.5 0 0"/></joint>
</robot>
"""


@pytest.fixture()
def urdf(tmp_path):
    p = tmp_path / 'robot.urdf'
    p.write_text(URDF)
    return p


def test_fk_planar_arm(urdf):
    kin = mm.UrdfKinematics(urdf)
    assert kin.dof == ['j1', 'j2']
    t = kin.link_pose('tip', {'j1': 0.0, 'j2': 0.0})
    # j1 origin has a 0.1 rad yaw, so at q=0 the chain points along the yaw direction
    np.testing.assert_allclose(t[:3, 3], [math.cos(0.1), math.sin(0.1), 0.12], atol=1e-12)
    t = kin.link_pose('tip', {'j1': math.pi / 2 - 0.1, 'j2': 0.0})
    np.testing.assert_allclose(t[:3, 3], [0, 1.0, 0.12], atol=1e-12)


def test_all_poses_matches_link_pose(urdf):
    kin = mm.UrdfKinematics(urdf)
    q = {'j1': 0.4, 'j2': -0.7}
    pose = kin.all_poses(q)
    for link in kin.links:
        np.testing.assert_allclose(pose(link), kin.link_pose(link, q), atol=1e-12)


def test_limits_and_sampling(urdf):
    kin = mm.UrdfKinematics(urdf)
    assert kin.limits == {'j1': (-1.5, 1.5), 'j2': (-3.2, 3.2)}
    q = mm.sample_configs(kin, 500, seed=3)
    assert q.shape == (500, 2)
    assert q[:, 0].min() >= -1.5 and q[:, 0].max() <= 1.5 and q[:, 1].min() >= -3.2 and q[:, 1].max() <= 3.2
    np.testing.assert_array_equal(q, mm.sample_configs(kin, 500, seed=3))
    assert not np.array_equal(q, mm.sample_configs(kin, 500, seed=4))


def test_allowed_pairs_hops(urdf):
    kin = mm.UrdfKinematics(urdf)
    one = mm.allowed_pairs(kin, 1)
    assert frozenset(('a', 'rigid')) in one          # same rigid group
    assert frozenset(('a', 'b')) in one              # adjacent
    assert frozenset(('b', 'c')) in one
    assert frozenset(('rigid', 'b')) in one          # rigid link counts as part of a
    assert frozenset(('a', 'c')) not in one          # two joints apart
    assert frozenset(('a', 'tip')) not in one
    two = mm.allowed_pairs(kin, 2)
    assert frozenset(('a', 'c')) in two and frozenset(('rigid', 'tip')) in two


def test_sphere_overlap_and_allowed(urdf):
    kin = mm.UrdfKinematics(urdf)
    q = {'j1': 0.0, 'j2': 0.0}
    # one sphere on a at the origin; one on c placed (in c's frame) at the world origin: they coincide
    inv_c = np.linalg.inv(kin.link_pose('c', q))
    centre_in_c = (inv_c @ np.array([0, 0, 0, 1.0]))[:3]
    spheres = {'a': [(np.zeros(3), 0.1)], 'c': [(centre_in_c, 0.1)]}
    world = mm.sphere_world(kin, spheres, q)
    assert mm.overlapping_pairs(world, set()) == [('a', 'c')]
    assert mm.overlapping_pairs(world, {frozenset(('a', 'c'))}) == []
    # exactly touching spheres (distance == r1 + r2) are free; a margin turns them into a hit
    spheres['c'] = [(centre_in_c + inv_c[:3, :3] @ [0.2, 0, 0], 0.1)]
    touching = mm.sphere_world(kin, spheres, q)
    assert mm.overlapping_pairs(touching, set()) == []
    assert mm.overlapping_pairs(touching, set(), margin=0.01) == [('a', 'c')]


def test_verdict_rates():
    truth = np.array([1, 1, 0, 0, 0, 1, 0, 0], bool)
    other = np.array([1, 0, 0, 1, 1, 1, 0, 0], bool)
    r = mm.verdict_rates(truth, other)
    assert r['n'] == 8
    assert r['agree'] == pytest.approx(5 / 8)
    assert r['false_free'] == pytest.approx(1 / 8)
    assert r['false_blocked'] == pytest.approx(2 / 8)
    assert r['false_free_given_collision'] == pytest.approx(1 / 3)
    assert r['false_blocked_given_free'] == pytest.approx(2 / 5)
    assert r['truth_collision_rate'] == pytest.approx(3 / 8)
    assert r['other_collision_rate'] == pytest.approx(4 / 8)
    perfect = mm.verdict_rates(truth, truth)
    assert perfect['agree'] == 1 and perfect['false_free'] == 0 and perfect['false_blocked'] == 0


def test_pose_error():
    a = np.eye(4)
    b = np.eye(4)
    b[:3, :3] = mm.axis_angle_matrix([0, 0, 1], 0.3)
    b[:3, 3] = [0.003, 0.004, 0]
    dp, dr = mm.pose_error(a, b)
    assert dp == pytest.approx(0.005) and dr == pytest.approx(0.3)
    assert mm.pose_error(a, a) == (0.0, 0.0)


def test_ur10e_dh_zero_pose():
    t = mm.ur10e_dh_fk([0] * 6)
    # standard UR10e at all-zero joints: arm stretched out along the DH base x axis
    assert t[0, 3] == pytest.approx(-1.18425)
    assert abs(t[1, 3]) == pytest.approx(0.17415 + 0.11655)
    assert t[2, 3] == pytest.approx(0.1807 - 0.11985)
    flange = mm.ur10e_flange_pose([0] * 6)
    assert flange[0, 3] == pytest.approx(1.18425)  # URDF base is turned by pi about z
    np.testing.assert_allclose(flange[:3, :3].T @ flange[:3, :3], np.eye(3), atol=1e-12)


def test_dh_rotation_only_joint_one_keeps_radius():
    r0 = np.linalg.norm(mm.ur10e_dh_fk([0, 0.3, -0.4, 0.2, 0.5, 0.1])[:2, 3])
    r1 = np.linalg.norm(mm.ur10e_dh_fk([1.2, 0.3, -0.4, 0.2, 0.5, 0.1])[:2, 3])
    assert r0 == pytest.approx(r1)


def test_patch_urdf_injects_compiler(urdf):
    out = mm.patch_urdf(urdf)
    assert out != urdf and out.parent == urdf.parent
    text = out.read_text()
    assert 'fusestatic="false"' in text and 'discardvisual="true"' in text
    again = mm.patch_urdf(out, out)  # idempotent: one block only
    assert again.read_text().count('<mujoco') == 1
    assert urdf.read_text() == URDF  # source untouched


def test_foam_sphere_parser(tmp_path):
    p = tmp_path / 's.urdf'
    p.write_text("""<robot name="s"><link name="l"><collision><origin xyz="1 2 3"/><geometry><sphere radius="0.05"/></geometry></collision>
    <collision><origin xyz="0 0 0"/><geometry><sphere radius="0.1"/></geometry></collision></link></robot>""")
    s = mm.load_foam_spheres(p)
    assert len(s['l']) == 2 and s['l'][0][1] == 0.05
    np.testing.assert_allclose(s['l'][0][0], [1, 2, 3])


def test_curobo_sphere_parser(tmp_path):
    pytest.importorskip('yaml')
    p = tmp_path / 'c.yml'
    p.write_text("kinematics:\n  collision_spheres:\n    l:\n    - center: [0.1, 0.2, 0.3]\n      radius: 0.04\n")
    s = mm.load_curobo_spheres(p)
    np.testing.assert_allclose(s['l'][0][0], [0.1, 0.2, 0.3])
    assert s['l'][0][1] == 0.04


def test_curobo_sphere_parser_finds_the_key_anywhere_and_follows_a_file_reference(tmp_path):
    pytest.importorskip('yaml')
    spheres = "l:\n- center: [0.1, 0.2, 0.3]\n  radius: 0.04\n"
    (tmp_path / 'spheres.yml').write_text(spheres)
    deep = tmp_path / 'deep.yml'
    deep.write_text("robot_cfg:\n  note: x\n  kinematics:\n    base_link: a\n    cspace: {max_acceleration: 1}\n"
                    "    collision_spheres:\n      " + spheres.replace('\n', '\n      ').rstrip() + "\n")
    ref = tmp_path / 'ref.yml'
    ref.write_text("kinematics:\n  collision_spheres: spheres.yml\n")
    for cfg in (deep, ref):
        s = mm.load_curobo_spheres(cfg)
        np.testing.assert_allclose(s['l'][0][0], [0.1, 0.2, 0.3])


def test_curobo_dump_must_come_from_the_same_configurations():
    import mujoco_crosscheck as cc
    q = np.random.default_rng(0).uniform(-1, 1, (5, 2))
    dump = {'joint_names': np.array(['j1', 'j2']), 'position': np.zeros((6, 3)), 'q': np.vstack([q, q[:1]])}
    cc.check_dump(dump, ['j1', 'j2'], q)  # a longer dump is fine, only the first rows count
    with pytest.raises(SystemExit, match='configurations'):
        cc.check_dump({**dump, 'q': dump['q'] + 1e-3}, ['j1', 'j2'], q)
    with pytest.raises(SystemExit, match='no `q`'):
        cc.check_dump({k: v for k, v in dump.items() if k != 'q'}, ['j1', 'j2'], q)


def test_hull_vertices_thinning():
    pytest.importorskip('scipy')
    rng = np.random.default_rng(0)
    pts = rng.normal(size=(2000, 3))
    v = mm.hull_vertices(pts, 24)
    assert len(v) == 24
    # thinned vertices are a subset of the original hull vertices
    assert all(np.min(np.linalg.norm(pts - x, axis=1)) < 1e-12 for x in v)


def test_mujoco_matches_urdf_fk_and_limits(urdf):
    mujoco = pytest.importorskip('mujoco')
    kin = mm.UrdfKinematics(urdf)
    model = mm.load_mujoco(urdf)
    data = mujoco.MjData(model)
    adr = mm.mujoco_qpos_addresses(model, kin.dof)
    for j, (lo, hi) in mm.mujoco_joint_ranges(model, kin.dof).items():
        assert (lo, hi) == pytest.approx(kin.limits[j])
    for q in mm.sample_configs(kin, 20, seed=1):
        data.qpos[adr] = q
        mujoco.mj_forward(model, data)
        err = mm.pose_error(mm.mujoco_pose(model, data, 'tip'), kin.link_pose('tip', dict(zip(kin.dof, q))))
        assert err[0] < 1e-9 and err[1] < 1e-7


def test_mujoco_contacts_respect_allowed_pairs(urdf):
    mujoco = pytest.importorskip('mujoco')
    kin = mm.UrdfKinematics(urdf)
    model = mm.load_mujoco(urdf)
    data = mujoco.MjData(model)
    adr = mm.mujoco_qpos_addresses(model, kin.dof)
    # stretched out: nothing touches
    data.qpos[adr] = [0.0, 0.0]
    mujoco.mj_forward(model, data)
    assert mm.mujoco_colliding_pairs(model, data, mm.allowed_pairs(kin, 1)) == []
    # j2 = pi folds c back over b and into the base box a (two joints from a)
    data.qpos[adr] = [0.0, math.pi]
    mujoco.mj_forward(model, data)
    assert mm.mujoco_colliding_pairs(model, data, mm.allowed_pairs(kin, 1)) == [('a', 'c')]
    assert mm.mujoco_colliding_pairs(model, data, mm.allowed_pairs(kin, 2)) == []
