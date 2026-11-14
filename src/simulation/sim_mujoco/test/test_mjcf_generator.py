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
