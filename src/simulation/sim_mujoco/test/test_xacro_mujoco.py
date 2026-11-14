"""hardware:=mujoco in the description xacro, and the profile-driven generator (needs ROS + xacro)."""
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest

pytest.importorskip('ament_index_python')
if shutil.which('xacro') is None:
    pytest.skip('xacro not available', allow_module_level=True)

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory  # noqa: E402

from sim_mujoco.generate import command_joints, load_profile, render_urdf  # noqa: E402

try:
    DESC = get_package_share_directory('mobile_manipulator_description')
except PackageNotFoundError:
    pytest.skip('mobile_manipulator_description not installed', allow_module_level=True)


def render(hardware):
    out = subprocess.run(['xacro', f'{DESC}/urdf/mobile_manipulator.urdf.xacro', 'arm:=generic_6dof',
                          f'hardware:={hardware}'], check=True, capture_output=True, text=True).stdout
    return ET.fromstring(out)


def test_mujoco_branch_has_plugin_joints_and_ft_sensor():
    root = ET.fromstring(subprocess.run(
        ['xacro', f'{DESC}/urdf/mobile_manipulator.urdf.xacro', 'arm:=generic_6dof', 'hardware:=mujoco'],
        check=True, capture_output=True, text=True).stdout)
    systems = [r for r in root.iter('ros2_control') if r.get('type') == 'system']
    assert len(systems) == 1
    system = systems[0]
    assert system.find('hardware/plugin').text == 'sim_mujoco/MujocoSystemInterface'
    joints = system.findall('joint')
    assert len(joints) == 7
    for j in joints:
        assert [c.get('name') for c in j.findall('command_interface')] == ['position', 'velocity']
    sensor = system.find('sensor')
    assert sensor.get('name') == 'ft_sensor'
    assert [s.get('name') for s in sensor.findall('state_interface')] == [
        'force.x', 'force.y', 'force.z', 'torque.x', 'torque.y', 'torque.z']


def test_other_backends_are_unchanged():
    mock = render('mock')
    assert mock.find('.//ros2_control/hardware/plugin').text == 'mock_components/GenericSystem'
    assert not list(mock.iter('sensor'))
    jtac = render('jtac_mock')
    assert len([r for r in jtac.iter('ros2_control')]) == 2


def test_profile_drives_joint_names_and_generation(tmp_path):
    profile = load_profile('generic_6dof_telescopic')
    assert command_joints(profile)[0] == 'lift_joint'
    from sim_mujoco.generate import generate
    path, _ = generate(profile, render_urdf(profile), tmp_path)
    root = ET.parse(path).getroot()
    assert [a.get('joint') for a in root.find('actuator')] == command_joints(profile)
    assert root.find('keyframe/key').get('name') == 'home'
