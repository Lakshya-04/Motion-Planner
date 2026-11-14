"""hardware:=gazebo rendering of the description (needs xacro and both packages on the ament path)."""
import os
import sys
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

xacro = pytest.importorskip('xacro')
ament = pytest.importorskip('ament_index_python.packages')
try:
    DESC = ament.get_package_share_directory('mobile_manipulator_description')
    ament.get_package_share_directory('sim_gazebo')
except Exception:  # noqa: BLE001
    pytest.skip('description / sim_gazebo not installed', allow_module_level=True)

from sim_gazebo import profile as prof  # noqa: E402

JOINT_CMD = {'position', 'velocity'}
JOINT_STATE = {'position', 'velocity', 'effort'}


def _render(profile_name, hardware):
    profile = prof.load_profile(profile_name)
    doc = xacro.process_file(os.path.join(DESC, 'urdf', 'mobile_manipulator.urdf.xacro'),
                             mappings=prof.xacro_mappings(profile, hardware))
    return profile, ET.fromstring(doc.toxml())


@pytest.mark.parametrize('profile_name', ['generic_6dof_telescopic'])
def test_gazebo_branch(profile_name):
    profile, robot = _render(profile_name, 'gazebo')
    rc = robot.find("ros2_control[@type='system']")
    assert rc.find('hardware/plugin').text == 'gz_ros2_control/GazeboSimSystem'
    joints = {j.get('name'): j for j in rc.findall('joint')}
    assert list(joints) == prof.controlled_joints(profile)
    for j in joints.values():
        assert {c.get('name') for c in j.findall('command_interface')} == JOINT_CMD
        assert {s.get('name') for s in j.findall('state_interface')} == JOINT_STATE
    sensor = rc.find("sensor[@name='ft_sensor']")
    assert {s.get('name') for s in sensor.findall('state_interface')} == {
        'force.x', 'force.y', 'force.z', 'torque.x', 'torque.y', 'torque.z'}

    plugin = robot.find('gazebo/plugin')
    assert plugin.get('filename') == 'gz_ros2_control-system'
    assert os.path.isfile(plugin.find('parameters').text)

    ft = robot.find("gazebo[@reference='ft_sensor_joint']")
    assert ft.find('preserveFixedJoint').text == 'true'
    assert ft.find("sensor[@type='force_torque']").get('name') == 'ft_sensor'

    # the sensor sits between the flange and the tool, on the flange (same kinematics as other backends)
    tool = next(j for j in robot.findall('joint') if j.find('child').get('link') == 'tool_mount_link')
    assert tool.find('parent').get('link') == 'ft_sensor_link'
    assert robot.find("gazebo[@reference='tool_mount_link']/sensor[@type='contact']") is not None

    pinned = [j for j in robot.findall('joint') if j.find('parent').get('link') == 'world']
    assert [j.find('child').get('link') for j in pinned] == [profile['frames']['world_base']]


@pytest.mark.parametrize('hardware', ['mock', 'jtac_mock'])
def test_other_backends_unchanged(hardware):
    _, robot = _render('generic_6dof_telescopic', hardware)
    assert robot.find('gazebo') is None
    assert robot.find("link[@name='world']") is None
    assert robot.find("link[@name='ft_sensor_link']") is None
    want_cmd = {'position', 'velocity'} if hardware == 'jtac_mock' else {'position'}
    for j in robot.find("ros2_control[@type='system']").findall('joint'):
        assert {s.get('name') for s in j.findall('state_interface')} == {'position', 'velocity'}
        assert {c.get('name') for c in j.findall('command_interface')} == want_cmd
