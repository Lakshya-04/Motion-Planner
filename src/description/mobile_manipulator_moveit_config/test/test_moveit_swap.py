"""Arm swap test through MoveIt: for each profile build the MoveIt config, check the planning
groups, joint names and DOF against the profile (read with the profile loader), and run an FK/IK
round trip on the whole-body group with the KDL plugin (test/fk_ik_check.cpp)."""
import os
import subprocess
import tempfile

import pytest
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from mobile_manipulator_description import load_profile
from mobile_manipulator_moveit_config import build_moveit_config, controllers_file, resolve_profile

PROFILES = ['generic_6dof_telescopic', 'panda_telescopic']
try:
    get_package_share_directory('ur_description')
    PROFILES.append('ur10e_telescopic')
except Exception:  # noqa: BLE001
    pass


def _tool():
    path = os.path.join(get_package_prefix('mobile_manipulator_moveit_config'), 'lib',
                        'mobile_manipulator_moveit_config', 'fk_ik_check')
    if not os.path.isfile(path):
        pytest.skip('fk_ik_check not built')
    return path


@pytest.mark.parametrize('name', PROFILES)
def test_moveit_config_follows_the_profile(name):
    p = load_profile(name)
    cfg = build_moveit_config(profile=name, ik='kdl')
    assert resolve_profile(profile=name).name == name
    import xml.etree.ElementTree as ET
    srdf = ET.fromstring(cfg.robot_description_semantic['robot_description_semantic'])
    assert {g.get('name') for g in srdf.findall('group')} == {
        p.arm_group, p.whole_body_group, p.tool_group}
    assert set(cfg.robot_description_kinematics['robot_description_kinematics']) >= {p.arm_group, p.whole_body_group}
    urdf = ET.fromstring(cfg.robot_description['robot_description'])
    system = urdf.find('ros2_control')
    assert [j.get('name') for j in system.findall('joint')] == p.joints


@pytest.mark.parametrize('name', PROFILES)
def test_controller_files_list_the_profile_joints(name):
    import yaml
    p = load_profile(name)
    for kind in ('trajectory', 'jtac'):
        data = yaml.safe_load(open(controllers_file(profile=name, controller=kind)))
        assert data['lift_arm_controller']['ros__parameters']['joints'] == p.joints
    moveit = yaml.safe_load(open(os.path.join(
        get_package_share_directory('mobile_manipulator_moveit_config'), 'config',
        p.controller_family, 'moveit_controllers.yaml')))
    assert moveit['moveit_simple_controller_manager']['lift_arm_controller']['joints'] == p.joints


@pytest.mark.parametrize('name', PROFILES)
def test_fk_ik_through_moveit(name):
    p = load_profile(name)
    cfg = build_moveit_config(profile=name, ik='kdl')
    with tempfile.TemporaryDirectory() as tmp:
        urdf = os.path.join(tmp, 'robot.urdf')
        srdf = os.path.join(tmp, 'robot.srdf')
        open(urdf, 'w').write(cfg.robot_description['robot_description'])
        open(srdf, 'w').write(cfg.robot_description_semantic['robot_description_semantic'])
        for group, dof, joints in ((p.whole_body_group, p.dof, p.joints),
                                   (p.arm_group, p.arm_dof, p.arm_joints)):
            out = subprocess.run([_tool(), urdf, srdf, group, '20'], capture_output=True,
                                 text=True, timeout=300)
            assert out.returncode == 0, out.stderr
            got = dict(line.split('=', 1) for line in out.stdout.split() if '=' in line)
            assert int(got['dof']) == dof
            assert got['joints'].split(',') == joints
            assert got['tip'] == p.tcp_link
            # IK from a nearby seed must recover the pose for most samples.
            assert int(got['ik_ok']) >= 12, out.stdout
            assert float(got['max_pose_error']) < 0.01, out.stdout
