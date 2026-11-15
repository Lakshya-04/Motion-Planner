"""hardware:=mujoco in the description xacro, and the profile-driven generator (needs ROS + xacro)."""
import pathlib
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest

try:
    from ament_index_python.packages import get_package_share_directory
    DESC = get_package_share_directory('mobile_manipulator_description')
except Exception:  # no ROS environment, or the description is not installed
    DESC = None
needs_ros = pytest.mark.skipif(DESC is None or shutil.which('xacro') is None,
                               reason='needs a sourced ROS environment with xacro and the description')


def render(hardware):
    out = subprocess.run(['xacro', f'{DESC}/urdf/mobile_manipulator.urdf.xacro', 'arm:=generic_6dof',
                          f'hardware:={hardware}'], check=True, capture_output=True, text=True).stdout
    return ET.fromstring(out)


@needs_ros
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


@needs_ros
def test_other_backends_are_unchanged():
    mock = render('mock')
    assert mock.find('.//ros2_control/hardware/plugin').text == 'mock_components/GenericSystem'
    assert not list(mock.iter('sensor'))
    jtac = render('jtac_mock')
    assert len([r for r in jtac.iter('ros2_control')]) == 2


@needs_ros
def test_profile_drives_joint_names_and_generation(tmp_path):
    from sim_mujoco.generate import command_joints, generate, load_profile, render_urdf
    profile = load_profile('generic_6dof_telescopic')
    assert command_joints(profile)[0] == 'lift_joint'
    path, _ = generate(profile, render_urdf(profile), tmp_path)
    root = ET.parse(path).getroot()
    assert [a.get('joint') for a in root.find('actuator')] == command_joints(profile)
    assert root.find('keyframe/key').get('name') == 'home'


@needs_ros
def test_nolift_profile_has_only_arm_commanded_joints(tmp_path):
    from sim_mujoco.generate import command_joints, generate, load_profile, render_urdf
    profile = load_profile('ur10e_nolift')
    assert profile['joints']['lift'] is None
    assert command_joints(profile) == profile['joints']['arm']
    path, _ = generate(profile, render_urdf(profile), tmp_path)
    root = ET.parse(path).getroot()
    assert [a.get('joint') for a in root.find('actuator')] == profile['joints']['arm']


JTAC_YAML = (pathlib.Path(__file__).resolve().parents[3] / 'control' / 'compliant_trajectory_controller'
             / 'config' / 'jtac_controller.yaml')


@needs_ros
def test_jtac_launch_params_match_the_raw_wrench_the_plugin_publishes():
    import yaml
    from sim_mujoco.generate import jtac_overrides, load_profile, render_urdf, tool_payload
    profile = load_profile('ur10e_telescopic')
    mass, cog = tool_payload(render_urdf(profile), profile['frames']['tool_mount'])
    assert mass == pytest.approx(0.5)
    p = jtac_overrides(yaml.safe_load(JTAC_YAML.read_text()), ['j'], '<robot/>', profile['frames'],
                       'ft_sensor', mass, cog)['combined_arm_lift_controller']['ros__parameters']
    # the wrench is raw at the mount: sensor and gravity frames are the mount, control is the TCP
    assert p['ft_sensor']['frame']['id'] == profile['frames']['tool_mount']
    assert p['gravity_compensation']['frame']['id'] == profile['frames']['tool_mount']
    assert p['control']['frame']['id'] == profile['frames']['tcp']
    assert p['gravity_compensation']['CoG']['force'] == pytest.approx(0.5 * 9.81)
    assert p['gravity_compensation']['CoG']['pos'] == pytest.approx(cog)


def test_tool_payload_composes_the_link_tree():
    from sim_mujoco.generate import tool_payload
    urdf = """<robot name="r"><link name="m"><inertial><origin xyz="0 0 0.1"/><mass value="1"/></inertial></link>
      <link name="t"><inertial><origin xyz="0 0 0"/><mass value="3"/></inertial></link>
      <joint name="j" type="fixed"><parent link="m"/><child link="t"/>
      <origin xyz="0.1 0 0" rpy="0 0 1.5707963"/></joint></robot>"""
    mass, cog = tool_payload(urdf, 'm')
    assert mass == pytest.approx(4.0)
    assert cog == pytest.approx([0.075, 0.0, 0.025])
