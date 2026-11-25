"""profile_reach.py: wall placement, tool orientation and joint names per robot profile (pure Python)."""
import importlib.util
import math
import pathlib
import subprocess
import sys

import pytest

HERE = pathlib.Path(__file__).parent
spec = importlib.util.spec_from_file_location('profile_reach', HERE.parent / 'scripts' / 'profile_reach.py')
pr = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pr)
SEVEN_DOF = ['panda_telescopic', 'fr3_telescopic', 'iiwa7_telescopic', 'kinova_gen3_telescopic', 'xarm7_telescopic']


def load(name):
    return pr.load_profile(name)


def test_stock_profile_reproduces_the_stock_wall():
    d = pr.derive(load('ur10e_telescopic'))
    assert d['wall_x'] == 0.98 and d['wall_dx'] == 0.0
    assert (d['z_lo'], d['z_hi'], d['wall_dz']) == (1.0, 1.3, 0.0)
    assert d['tool_yaw_deg'] == -90.0
    assert d['free_pose'][:3] == pytest.approx([0.95, 0.15, 1.15])


def test_stock_tool_orientation_is_a_quarter_turn_about_y():
    q = pr.tool_quaternion(-90.0)
    assert q == pytest.approx((0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5)))


@pytest.mark.parametrize('yaw', [-180.0, -90.0, 0.0, 45.0, 90.0, 170.0])
def test_tool_quaternion_is_unit_and_points_the_tool_axis_at_the_wall(yaw):
    qx, qy, qz, qw = pr.tool_quaternion(yaw)
    assert math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) == pytest.approx(1.0)
    # R[0][2] = 2(xz + wy) is the x component of the tool z axis: +x, into the wall
    assert 2 * (qx * qz + qw * qy) == pytest.approx(1.0)


def test_shorter_arm_gets_a_nearer_wall_and_the_ordering_follows_reach():
    walls = {n: pr.derive(load(n))['wall_x'] for n in SEVEN_DOF + ['ur10e_telescopic', 'ur5e_telescopic']}
    assert walls['xarm7_telescopic'] <= walls['panda_telescopic'] <= walls['ur10e_telescopic']
    assert walls['ur5e_telescopic'] < walls['ur10e_telescopic']
    assert walls['kinova_gen3_telescopic'] > walls['iiwa7_telescopic']


@pytest.mark.parametrize('name', SEVEN_DOF)
def test_seven_dof_profiles_derive(name):
    p = load(name)
    d = pr.derive(p)
    assert 0.4 < d['wall_x'] < 1.0
    front = max(pt[0] for pt in p['base']['footprint'])
    assert d['wall_x'] >= front + pr.MIN_BASE_CLEARANCE_M - 1e-9
    assert d['z_hi'] - d['z_lo'] == pytest.approx(0.3)
    assert len(pr.joint_names(p)) == 8 and pr.joint_names(p)[0] == 'lift_joint'
    assert d['free_pose'][0] == pytest.approx(d['wall_x'] - pr.FREE_STANDOFF_M)


def test_joint_names_of_the_stock_profile_are_the_ur_ones():
    assert pr.joint_names(load('ur10e_telescopic')) == [
        'lift_joint', 'arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_joint',
        'arm_wrist_1_joint', 'arm_wrist_2_joint', 'arm_wrist_3_joint']


def test_standoff_moves_only_the_free_pose_x():
    a, b = pr.derive(load('panda_telescopic'), 0.03), pr.derive(load('panda_telescopic'), 0.30)
    assert a['wall_x'] == b['wall_x']
    assert a['free_pose'][0] - b['free_pose'][0] == pytest.approx(0.27)


def test_unknown_reach_and_too_short_reach_are_errors():
    with pytest.raises(ValueError):
        pr.derive(load('generic_6dof_telescopic'))
    assert pr.derive(load('generic_6dof_telescopic'), reach=0.9)['wall_x'] > 0.4
    with pytest.raises(ValueError):
        pr.derive(load('ur3e_telescopic'))


def test_profile_can_override_the_tool_yaw():
    p = load('panda_telescopic')
    assert pr.derive(p)['tool_yaw_deg'] == -90.0
    p['coverage'] = {'tool_frame_yaw_deg': 90.0}
    assert pr.derive(p)['tool_yaw_deg'] == 90.0


def test_shell_output_is_evalable_assignments():
    lines = pr.shell_lines(pr.derive(load('panda_telescopic')))
    names = [ln.split('=')[0] for ln in lines]
    assert {'MP_WALL_X', 'MP_WALL_DX', 'MP_Z_LO', 'MP_Z_HI', 'MP_WALL_DZ', 'MP_TOOL_YAW', 'MP_FREE_POSE'} <= set(names)


SCRIPT = HERE.parent / 'scripts' / 'profile_reach.py'


def test_shell_output_evals_in_bash_under_nounset():
    out = subprocess.run([sys.executable, str(SCRIPT), '--profile', 'panda_telescopic', '--shell'],
                         capture_output=True, text=True, check=True).stdout
    script = f'set -u\nvars=$(cat <<\'EOF\'\n{out}EOF\n)\neval "$vars"\necho "$MP_WALL_X|$MP_TOOL_YAW|$MP_ARM"\n'
    r = subprocess.run(['bash', '-c', script], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    wall_x, yaw, arm = r.stdout.strip().split('|')
    d = pr.derive(load('panda_telescopic'))
    assert float(wall_x) == pytest.approx(d['wall_x']) and float(yaw) == -90.0 and arm == 'panda'


@pytest.mark.parametrize('name', ['ur3e_telescopic', 'generic_6dof_telescopic', 'no_such_profile'])
def test_failing_profile_exits_nonzero_with_a_message_and_prints_nothing_evalable(name):
    r = subprocess.run([sys.executable, str(SCRIPT), '--profile', name, '--shell'], capture_output=True, text=True)
    assert r.returncode == 2
    assert r.stdout == ''
    assert f'profile {name}' in r.stderr


def test_missing_lift_mount_height_falls_back_to_the_xacro_default_or_names_the_key():
    p = load('ur10e_husky_telescopic')
    assert 'mount_height_m' not in p['lift']
    assert pr.mount_z(p) == pytest.approx(p['base']['lift_mount_height_m'] + 0.82)
    p['lift']['type'] = 'scissor'
    with pytest.raises(ValueError, match='lift.mount_height_m'):
        pr.mount_z(p)
