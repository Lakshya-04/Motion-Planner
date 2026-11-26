"""MJCF generator tests. Pure-Python parts run anywhere; the compile checks need `mujoco` (sim image)."""
import math
import pathlib
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from sim_mujoco.mjcf_generator import UrdfRobot, quat_from_rpy, urdf_to_mjcf

DATA = pathlib.Path(__file__).parent / 'data'
URDF = DATA / 'test_arm.urdf'
HOME = {'lift_joint': 0.2, 'joint1': 0.1, 'joint2': -0.4}
COMMANDED = ['lift_joint', 'joint1', 'joint2']


@pytest.fixture(scope='module')
def mjcf(tmp_path_factory):
    out = tmp_path_factory.mktemp('mjcf')
    path = urdf_to_mjcf(URDF.read_text(), out, urdf_dir=DATA, home=HOME, commanded=COMMANDED)
    return path


def rot(axis, a):
    c, s = math.cos(a), math.sin(a)
    return {'x': np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
            'y': np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            'z': np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def tf(xyz, R=np.eye(3)):
    t = np.eye(4)
    t[:3, :3] = R
    t[:3, 3] = xyz
    return t


def spec_tcp(q_lift, q1, q2):
    """tcp pose written out by hand from test_arm.urdf."""
    t = tf([0, 0, 0.1 + q_lift])
    t = t @ tf([0, 0, 0.2], rot('z', 0.3)) @ tf([0, 0, 0], rot('z', q1))
    t = t @ tf([0.4, 0, 0], rot('y', 0.2)) @ tf([0, 0, 0], rot('y', q2))
    t = t @ tf([0.3, 0, 0], rot('y', 1.5708)) @ tf([0, 0, 0.165])
    return t


def test_quat_from_rpy_matches_urdf_convention():
    # extrinsic x then y then z: rotate x axis by roll 90deg about x leaves it, yaw then moves it
    w, x, y, z = quat_from_rpy(0.0, 0.0, math.pi / 2)
    assert (w, x, y, z) == pytest.approx((math.sqrt(0.5), 0, 0, math.sqrt(0.5)))
    q = quat_from_rpy(0.4, -0.3, 1.1)
    assert sum(c * c for c in q) == pytest.approx(1.0)


def test_urdf_parse():
    robot = UrdfRobot(URDF.read_text())
    assert robot.root == 'base_link'
    assert robot.joint_by_child['stage_link'].mimic == ('lift_joint', 0.5, 0.0)
    # tool mount and tcp are one rigid group with link2
    assert robot.group_of('tcp_link') == robot.group_of('tool_mount_link') == 'link2'


def test_ignore_hops():
    robot = UrdfRobot(URDF.read_text())
    pairs = {frozenset(p) for p in robot.ignored_pairs(['base_link', 'lift_link', 'link1', 'link2'], 1)}
    assert frozenset(('lift_link', 'link1')) in pairs
    assert frozenset(('base_link', 'link1')) not in pairs
    pairs2 = {frozenset(p) for p in robot.ignored_pairs(['base_link', 'lift_link', 'link1', 'link2'], 2)}
    assert frozenset(('base_link', 'link1')) in pairs2


def test_mimic_stage_adds_no_hop():
    """Telescope stages move together: a mimic joint is zero hops, so a stage is as close as its driver."""
    robot = UrdfRobot(URDF.read_text())
    assert robot.tree_distance('stage_link', 'lift_link') == 1
    assert robot.tree_distance('stage_link', 'base_link') == 0
    pairs = {frozenset(p) for p in robot.ignored_pairs(['base_link', 'lift_link', 'stage_link', 'link1'], 1)}
    assert frozenset(('stage_link', 'lift_link')) in pairs
    assert frozenset(('stage_link', 'link1')) not in pairs


def test_generated_structure(mjcf):
    root = ET.parse(mjcf).getroot()
    assert [j.get('name') for j in root.iter('joint') if j.get('type') in ('hinge', 'slide')] == \
        ['lift_joint', 'joint1', 'joint2', 'stage_joint']  # depth-first, not URDF order
    acts = root.find('actuator')
    assert [a.get('joint') for a in acts] == COMMANDED  # the mimic stage is not commanded
    assert root.find('equality').find('joint').get('polycoef').split()[:2] == ['0', '0.5']
    assert {s.get('name') for s in root.iter('site')} == {'ft_sensor', 'tcp_site'}
    assert {s.get('site') for s in root.find('sensor')} == {'ft_sensor'}
    assert (mjcf.parent / 'meshes').is_dir() and len(list((mjcf.parent / 'meshes').glob('*.stl'))) == 1
    force = [float(v) for v in acts[1].get('forcerange').split()]
    assert force == [-100.0, 100.0]  # joint1 effort limit
    # everything beyond the F/T site keeps its weight, the rest is gravity compensated
    bodies = {b.get('name'): b for b in root.iter('body')}
    assert bodies['link1'].get('gravcomp') == '1'
    assert bodies['tool_mount_link'].get('gravcomp') is None
    assert bodies['tcp_link'].get('gravcomp') is None
    # the tool is offset from the sensor: the pad sits at the TCP, 0.165 m along the tool axis
    pad = [g for g in bodies['tool_mount_link'].findall('geom') if g.get('name') == 'tool_pad'][0]
    assert float(pad.get('pos').split()[2]) == pytest.approx(0.165 - 0.005)


def test_pad_sits_behind_the_tcp_along_the_tcp_own_axis(tmp_path):
    text = URDF.read_text().replace('<origin xyz="0 0 0.165"/>', '<origin xyz="0 0 0.165" rpy="0 0.5 0"/>')
    root = ET.parse(urdf_to_mjcf(text, tmp_path, urdf_dir=DATA, commanded=COMMANDED)).getroot()
    mount = [b for b in root.iter('body') if b.get('name') == 'tool_mount_link'][0]
    pad = [g for g in mount.findall('geom') if g.get('name') == 'tool_pad'][0]
    axis = rot('y', 0.5) @ np.array([0, 0, 1.0])
    expected = np.array([0, 0, 0.165]) - axis * 0.005
    assert [float(v) for v in pad.get('pos').split()] == pytest.approx(list(expected), abs=1e-6)


def test_mesh_formats_are_checked(tmp_path):
    text = URDF.read_text().replace('cube.stl', 'cube.dae')
    with pytest.raises(ValueError, match='STL/OBJ'):
        urdf_to_mjcf(text, tmp_path, urdf_dir=DATA)


def test_unknown_option_rejected(tmp_path):
    with pytest.raises(TypeError):
        urdf_to_mjcf(URDF.read_text(), tmp_path, urdf_dir=DATA, bogus=1)


try:
    import mujoco
except ImportError:  # the build image has no MuJoCo python package
    mujoco = None
needs_mujoco = pytest.mark.skipif(mujoco is None, reason='mujoco python package not installed')


def load(path):
    return mujoco.MjModel.from_xml_path(str(path))


@needs_mujoco
def test_compiles_with_joint_limits_and_keyframe(mjcf):
    m = load(mjcf)
    assert m.njnt == 4 and m.nu == 3 and m.nsensor == 2
    rng = m.jnt_range[m.joint('joint1').id]
    assert tuple(rng) == pytest.approx((-3.0, 3.0))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    assert d.qpos[m.jnt_qposadr[m.joint('stage_joint').id]] == pytest.approx(0.1)  # 0.5 * lift 0.2
    assert d.qpos[m.jnt_qposadr[m.joint('joint2').id]] == pytest.approx(-0.4)


@needs_mujoco
def test_forward_kinematics_matches_urdf_spec(mjcf):
    m = load(mjcf)
    d = mujoco.MjData(m)
    rng = np.random.default_rng(1)
    for _ in range(20):
        ql, q1, q2 = rng.uniform(0, 0.4), rng.uniform(-2.5, 2.5), rng.uniform(-1.8, 1.8)
        d.qpos[:] = 0
        d.qpos[m.jnt_qposadr[m.joint('lift_joint').id]] = ql
        d.qpos[m.jnt_qposadr[m.joint('joint1').id]] = q1
        d.qpos[m.jnt_qposadr[m.joint('joint2').id]] = q2
        mujoco.mj_kinematics(m, d)
        t = spec_tcp(ql, q1, q2)
        s = d.site('tcp_site')
        assert s.xpos == pytest.approx(t[:3, 3], abs=1e-9)
        assert s.xmat.reshape(3, 3) == pytest.approx(t[:3, :3], abs=1e-9)


@needs_mujoco
def test_inertial_is_carried_over(mjcf):
    m = load(mjcf)
    i = m.body('link1').id
    assert m.body_mass[i] == pytest.approx(3.0)
    assert m.body_ipos[i] == pytest.approx([0.2, 0, 0])


@needs_mujoco
def test_position_actuator_tracks_and_gravity_is_held(mjcf):
    m = load(mjcf)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    target = {'lift_joint': 0.3, 'joint1': -0.5, 'joint2': 0.6}
    for k, name in enumerate(COMMANDED):
        d.ctrl[k] = target[name]
    for _ in range(int(4.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    for name, v in target.items():
        err = abs(d.qpos[m.jnt_qposadr[m.joint(name).id]] - v)
        assert err < 0.01, (name, err)  # tool weight beyond the gravity-compensated chain sags a little
    # the mimic stage follows the lift
    assert d.qpos[m.jnt_qposadr[m.joint('stage_joint').id]] == pytest.approx(0.15, abs=0.01)


@needs_mujoco
def test_sensor_reads_tool_weight(mjcf):
    m = load(mjcf)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    d.ctrl[:] = [HOME[j] for j in COMMANDED]
    for _ in range(int(3.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    force = -d.sensor('ft_force').data  # sign as in MujocoSim: the tool side's load on the sensor
    mass = m.body_subtreemass[m.body('tool_mount_link').id]
    assert np.linalg.norm(force) == pytest.approx(mass * 9.81, rel=0.03)
    # at the sensor frame the weight vector is gravity rotated into that frame
    r = d.site('ft_sensor').xmat.reshape(3, 3)
    assert force == pytest.approx(r.T @ np.array([0, 0, -mass * 9.81]), abs=0.15)


# ------------------------------------------------------------------ planar base (base_motion)
BASE = {'drive': 'omni', 'max_linear': 0.4, 'max_angular': 0.8}


def test_base_motion_off_leaves_the_model_unchanged(mjcf):
    root = ET.parse(mjcf).getroot()
    assert root.find('custom') is None
    assert not [j for j in root.iter('joint') if j.get('name', '').startswith('base_')]


def test_base_motion_adds_joints_actuators_and_custom(tmp_path):
    path = urdf_to_mjcf(URDF.read_text(), tmp_path, urdf_dir=DATA, home=HOME, commanded=COMMANDED,
                        base_motion=BASE)
    root = ET.parse(path).getroot()
    base = [j for j in root.iter('joint') if j.get('name', '').startswith('base_')]
    assert [(j.get('name'), j.get('type'), j.get('axis')) for j in base] == [
        ('base_x_joint', 'slide', '1 0 0'), ('base_y_joint', 'slide', '0 1 0'),
        ('base_yaw_joint', 'hinge', '0 0 1')]
    root_body = root.find('worldbody').find('body')
    assert root_body.get('name') == 'base_link' and len(root_body.findall('joint')) == 3
    assert [a.get('joint') for a in root.find('actuator')] == COMMANDED + [j.get('name') for j in base]
    nums = {n.get('name'): n.get('data') for n in root.find('custom')}
    assert nums == {'sim_mujoco_base_diff': '0', 'sim_mujoco_base_max_vel': '0.4 0.8'}
    key = root.find('keyframe').find('key').get('qpos').split()
    assert key[:3] == ['0', '0', '0'] and len(key) == 3 + 4  # base first, then the 4 arm / lift joints


def test_base_motion_diff_and_bad_drive(tmp_path):
    path = urdf_to_mjcf(URDF.read_text(), tmp_path, urdf_dir=DATA, base_motion=dict(BASE, drive='diff'))
    nums = {n.get('name'): n.get('data') for n in ET.parse(path).getroot().find('custom')}
    assert nums['sim_mujoco_base_diff'] == '1'
    with pytest.raises(ValueError, match='omni'):
        urdf_to_mjcf(URDF.read_text(), tmp_path, urdf_dir=DATA, base_motion=dict(BASE, drive='4ws'))


def test_base_motion_options_from_profile():
    from sim_mujoco.generate import base_motion_options
    assert base_motion_options({'base': {'drive_type': 'skid', 'max_linear_velocity_mps': 1.2,
                                         'max_angular_velocity_radps': 2.0}}) == \
        dict(drive='diff', max_linear=1.2, max_angular=2.0)
    assert base_motion_options({}) == dict(drive='omni', max_linear=0.5, max_angular=1.0)
    assert base_motion_options({'base': {'drive_type': '4ws'}})['drive'] == 'omni'


@needs_mujoco
def test_base_moves_when_commanded_and_holds_otherwise(tmp_path):
    path = urdf_to_mjcf(URDF.read_text(), tmp_path, urdf_dir=DATA, home=HOME, commanded=COMMANDED,
                        base_motion=BASE)
    m = load(path)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    ix = m.actuator('act_base_x_joint').id
    qx, qy, qw = (m.jnt_qposadr[m.joint(f'base_{n}_joint').id] for n in ('x', 'y', 'yaw'))
    d.ctrl[:3] = [HOME[j] for j in COMMANDED]
    for _ in range(int(2.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    assert abs(d.qpos[qx]) < 1e-3 and abs(d.qpos[qy]) < 1e-3 and abs(d.qpos[qw]) < 1e-3  # holds
    target = 0.0
    for _ in range(int(2.0 / m.opt.timestep)):  # 0.2 m/s along x for 2 s
        target += 0.2 * m.opt.timestep
        d.ctrl[ix] = target
        mujoco.mj_step(m, d)
    assert d.qpos[qx] == pytest.approx(0.4, abs=0.03)
    assert abs(d.qpos[qy]) < 1e-3


# ------------------------------------------------------------------ depth sensor (depth_sensor)
def test_depth_sensor_off_leaves_the_model_unchanged(tmp_path, mjcf):
    path = urdf_to_mjcf(URDF.read_text(), tmp_path, urdf_dir=DATA, home=HOME, commanded=COMMANDED,
                        depth_sensor=None)
    assert path.read_text() == mjcf.read_text()
    root = ET.parse(path).getroot()
    assert not [s for s in root.iter('site') if s.get('name') == 'depth_sensor']


def test_depth_sensor_adds_site_numeric_and_scene_group(tmp_path):
    scene_geom = ET.Element('geom', name='wall', type='box', size='0.5 0.5 0.05')
    path = urdf_to_mjcf(URDF.read_text(), tmp_path, urdf_dir=DATA, home=HOME, commanded=COMMANDED,
                        world_geoms=[scene_geom],
                        depth_sensor=dict(pos=(0.1, 0.0, 1.2), rpy=(0.0, 0.0, math.pi / 2), noise_std=0.004))
    root = ET.parse(path).getroot()
    root_body = root.find('worldbody').find('body')
    site = [s for s in root_body.findall('site') if s.get('name') == 'depth_sensor'][0]
    assert site.get('pos').split() == ['0.1', '0', '1.2']
    q = [float(v) for v in site.get('quat').split()]
    assert q == pytest.approx([math.sqrt(0.5), 0, 0, math.sqrt(0.5)])
    nums = {n.get('name'): [float(v) for v in n.get('data').split()] for n in root.find('custom')}
    assert nums['sim_mujoco_depth'] == pytest.approx(
        [96, 64, math.radians(60), math.radians(45), 0.1, 5.0, 5.0, 0.004])
    # the scene is in group 1 (the one the sensor scans); the robot's own geoms are not
    assert [g.get('group') for g in root.find('worldbody').findall('geom')] == ['1']
    assert all(g.get('group') is None for g in root_body.iter('geom'))


def test_depth_sensor_with_base_motion_shares_one_custom_block(tmp_path):
    path = urdf_to_mjcf(URDF.read_text(), tmp_path, urdf_dir=DATA, base_motion=BASE,
                        depth_sensor=dict(pos=(0, 0, 1)))
    root = ET.parse(path).getroot()
    assert len(root.findall('custom')) == 1
    assert [n.get('name') for n in root.find('custom')] == [
        'sim_mujoco_base_diff', 'sim_mujoco_base_max_vel', 'sim_mujoco_depth']


@needs_mujoco
def test_depth_sensor_model_compiles_and_a_ray_hits_only_the_scene(tmp_path):
    scene_geom = ET.Element('geom', name='wall', type='box', size='0.05 1 1', pos='1.05 0 1')
    path = urdf_to_mjcf(URDF.read_text(), tmp_path, urdf_dir=DATA, home=HOME, commanded=COMMANDED,
                        world_geoms=[scene_geom], depth_sensor=dict(pos=(0, 0, 1)))
    m = load(path)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'depth_sensor')
    groups = np.array([0, 1, 0, 0, 0, 0], dtype=np.uint8)
    geomid = np.zeros(1, dtype=np.int32)
    dist = mujoco.mj_ray(m, d, d.site_xpos[site], d.site_xmat[site].reshape(3, 3)[:, 0], groups, 1, -1, geomid)
    assert dist == pytest.approx(1.0, abs=1e-6)
    assert m.geom(geomid[0]).name == 'wall'


# ------------------------------------------------------------------ lidar (lidar)
def test_lidar_off_leaves_the_model_unchanged(tmp_path, mjcf):
    path = urdf_to_mjcf(URDF.read_text(), tmp_path, urdf_dir=DATA, home=HOME, commanded=COMMANDED, lidar=None)
    assert path.read_text() == mjcf.read_text()
    assert not [s for s in ET.parse(path).getroot().iter('site') if s.get('name') == 'lidar']


def test_lidar_adds_site_numeric_and_scene_group(tmp_path):
    scene_geom = ET.Element('geom', name='wall', type='box', size='0.5 0.5 0.05')
    path = urdf_to_mjcf(URDF.read_text(), tmp_path, urdf_dir=DATA, home=HOME, commanded=COMMANDED,
                        world_geoms=[scene_geom],
                        lidar=dict(pos=(0.45, 0.0, 0.2), rpy=(0.0, 0.0, math.pi / 2), noise_std=0.01, dropout=0.05,
                                   seed=9))
    root = ET.parse(path).getroot()
    root_body = root.find('worldbody').find('body')
    site = [s for s in root_body.findall('site') if s.get('name') == 'lidar'][0]
    assert site.get('pos').split() == ['0.45', '0', '0.2']
    q = [float(v) for v in site.get('quat').split()]
    assert q == pytest.approx([math.sqrt(0.5), 0, 0, math.sqrt(0.5)])
    nums = {n.get('name'): [float(v) for v in n.get('data').split()] for n in root.find('custom')}
    assert nums['sim_mujoco_lidar'] == pytest.approx(
        [-0.75 * math.pi, 0.75 * math.pi, math.radians(0.5), 0.05, 10.0, 10.0, 0.01, 0.05, 9])
    assert [g.get('group') for g in root.find('worldbody').findall('geom')] == ['1']
    assert all(g.get('group') is None for g in root_body.iter('geom'))


def test_lidar_and_depth_and_base_share_one_custom_block(tmp_path):
    path = urdf_to_mjcf(URDF.read_text(), tmp_path, urdf_dir=DATA, base_motion=BASE,
                        depth_sensor=dict(pos=(0, 0, 1)), lidar=dict(pos=(0, 0, 0.2)))
    root = ET.parse(path).getroot()
    assert len(root.findall('custom')) == 1
    assert [n.get('name') for n in root.find('custom')] == [
        'sim_mujoco_base_diff', 'sim_mujoco_base_max_vel', 'sim_mujoco_depth', 'sim_mujoco_lidar']


def test_lidar_with_a_bad_value_is_refused(tmp_path):
    with pytest.raises(ValueError):
        urdf_to_mjcf(URDF.read_text(), tmp_path, urdf_dir=DATA, lidar=dict(pos=(0, 0, 0.2), dropout=2.0))


@needs_mujoco
def test_lidar_model_compiles_and_the_fan_hits_only_the_scene(tmp_path):
    scene_geom = ET.Element('geom', name='wall', type='box', size='0.05 3 1', pos='2.05 0 0.2')
    path = urdf_to_mjcf(URDF.read_text(), tmp_path, urdf_dir=DATA, home=HOME, commanded=COMMANDED,
                        world_geoms=[scene_geom], lidar=dict(pos=(0, 0, 0.2)))
    m = load(path)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'lidar')
    assert site >= 0
    groups = np.array([0, 1, 0, 0, 0, 0], dtype=np.uint8)
    geomid = np.zeros(1, dtype=np.int32)
    for ang in (0.0, 0.5, -0.8):
        v = d.site_xmat[site].reshape(3, 3) @ np.array([math.cos(ang), math.sin(ang), 0.0])
        dist = mujoco.mj_ray(m, d, d.site_xpos[site], v, groups, 1, -1, geomid)
        assert dist == pytest.approx((2.0 - d.site_xpos[site][0]) / math.cos(ang), abs=1e-6)
        assert m.geom(geomid[0]).name == 'wall'


# ---------------------------------------------------------------- per-body gravcomp


def _gravcomp_bodies(path):
    return {b.get('name') for b in ET.parse(path).getroot().iter('body') if b.get('gravcomp') == '1'}


def test_gravcomp_default_unchanged_and_bool_forms(tmp_path, mjcf):
    default = _gravcomp_bodies(mjcf)
    assert 'link1' in default and 'tool_mount_link' not in default and 'tcp_link' not in default
    on = urdf_to_mjcf(URDF.read_text(), tmp_path / 'on', urdf_dir=DATA, home=HOME,
                      commanded=COMMANDED, gravcomp=True)
    assert on.read_bytes() == mjcf.read_bytes()
    off = urdf_to_mjcf(URDF.read_text(), tmp_path / 'off', urdf_dir=DATA, home=HOME,
                       commanded=COMMANDED, gravcomp=False)
    assert _gravcomp_bodies(off) == set()


def test_gravcomp_per_link_list_selects_exactly_those(tmp_path):
    path = urdf_to_mjcf(URDF.read_text(), tmp_path, urdf_dir=DATA, home=HOME,
                        commanded=COMMANDED, gravcomp=['link1'])
    assert _gravcomp_bodies(path) == {'link1'}


def test_gravcomp_unknown_link_rejected(tmp_path):
    with pytest.raises(ValueError, match='unknown links'):
        urdf_to_mjcf(URDF.read_text(), tmp_path, urdf_dir=DATA, gravcomp=['nope'])


@pytest.mark.parametrize('names', [['link1'], ('link1',), {'link1'}, frozenset({'link1'})])
def test_gravcomp_accepts_any_non_string_iterable(tmp_path, names):
    path = urdf_to_mjcf(URDF.read_text(), tmp_path, urdf_dir=DATA, home=HOME,
                        commanded=COMMANDED, gravcomp=names)
    assert _gravcomp_bodies(path) == {'link1'}


@pytest.mark.parametrize('bad', [{'link1': 1}, 3, 1.5, b'link1', object()])
def test_gravcomp_rejects_dicts_and_other_types(tmp_path, bad):
    with pytest.raises(TypeError):
        urdf_to_mjcf(URDF.read_text(), tmp_path, urdf_dir=DATA, home=HOME,
                     commanded=COMMANDED, gravcomp=bad)


def test_a_urdf_pad_replaces_the_generated_one(tmp_path):
    # generic_tool.xacro carries the pad as a collision named tool_pad (so the planners see it): the generator must use
    # that one and not add a second, identical geom (two contacts per touch double the force)
    pad = ('<collision name="tool_pad"><origin xyz="0 0 0.16"/><geometry><cylinder radius="0.03" length="0.01"/>'
           '</geometry></collision>')
    text = URDF.read_text().replace('<link name="tool_mount_link">', '<link name="tool_mount_link">' + pad, 1)
    root = ET.parse(urdf_to_mjcf(text, tmp_path, urdf_dir=DATA, commanded=COMMANDED)).getroot()
    mount = [b for b in root.iter('body') if b.get('name') == 'tool_mount_link'][0]
    pads = [g for g in mount.findall('geom') if g.get('name') == 'tool_pad']
    assert len(pads) == 1
    assert [float(v) for v in pads[0].get('size').split()] == pytest.approx([0.03, 0.005])
    assert float(pads[0].get('pos').split()[2]) == pytest.approx(0.16)


# ---------------------------------------------------------------- visual-only floor
def test_floor_off_leaves_the_model_unchanged(tmp_path, mjcf):
    path = urdf_to_mjcf(URDF.read_text(), tmp_path, urdf_dir=DATA, home=HOME, commanded=COMMANDED, floor=False)
    assert path.read_text() == mjcf.read_text()
    assert 'floor' not in path.read_text()


def test_floor_is_visual_only_and_leaves_the_wall_alone(tmp_path):
    wall = ET.Element('geom', name='wall_0', type='box', size='0.05 0.5 1', pos='1 0 1', friction='1 0.005 0.0001')
    plain = urdf_to_mjcf(URDF.read_text(), tmp_path / 'a', urdf_dir=DATA, home=HOME, commanded=COMMANDED,
                         world_geoms=[ET.fromstring(ET.tostring(wall))], base_motion=BASE)
    with_floor = urdf_to_mjcf(URDF.read_text(), tmp_path / 'b', urdf_dir=DATA, home=HOME, commanded=COMMANDED,
                              world_geoms=[ET.fromstring(ET.tostring(wall))], base_motion=BASE, floor=True)
    world = ET.parse(with_floor).getroot().find('worldbody')
    floor = [g for g in world.findall('geom') if g.get('name') == 'floor'][0]
    assert floor.get('type') == 'plane'
    assert floor.get('contype') == '0' and floor.get('conaffinity') == '0' and floor.get('group') == '2'
    walls = [ET.tostring(g) for g in world.findall('geom') if g.get('name') == 'wall_0']
    assert walls == [ET.tostring(g) for g in ET.parse(plain).getroot().find('worldbody').findall('geom')
                     if g.get('name') == 'wall_0']
    assert len(walls) == 1


@needs_mujoco
def test_floor_adds_no_contacts_and_is_not_hit_by_scene_rays(tmp_path):
    wall = ET.Element('geom', name='wall', type='box', size='0.05 1 1', pos='1.05 0 1')
    path = urdf_to_mjcf(URDF.read_text(), tmp_path, urdf_dir=DATA, home=HOME, commanded=COMMANDED,
                        world_geoms=[wall], depth_sensor=dict(pos=(0, 0, 1)), floor=True)
    m = load(path)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    assert d.ncon == 0
    floor_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
    assert m.geom_contype[floor_id] == 0 and m.geom_conaffinity[floor_id] == 0
    geomid = np.zeros(1, dtype=np.int32)
    groups = np.array([0, 1, 0, 0, 0, 0], dtype=np.uint8)
    dist = mujoco.mj_ray(m, d, np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, -1.0]), groups, 1, -1, geomid)
    assert dist == -1
