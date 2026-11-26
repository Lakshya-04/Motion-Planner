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
    # commanded joints; the mimic lift stages are state-only and checked in test_gazebo_lift_stage_mimic
    joints = {j.get('name'): j for j in rc.findall('joint') if j.find('command_interface') is not None}
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
    assert robot.find("gazebo[@reference='tool_tip_link']/sensor[@type='contact']") is not None
    # the tool collision is on the spring-loaded tip, a side branch (not in the arm-to-tcp chain)
    assert robot.find("link[@name='tool_mount_link']/collision") is None
    tip = next(j for j in robot.findall('joint') if j.get('name') == 'tool_tip_joint')
    assert tip.get('type') == 'prismatic' and tip.find('parent').get('link') == 'tool_mount_link'
    # bump stop: a stiff, short second stage in series carries the tip (bounded force beyond the soft travel)
    stop = next(j for j in robot.findall('joint') if j.get('name') == 'tool_tip_stop_joint')
    assert stop.get('type') == 'prismatic' and stop.find('child').get('link') == 'tool_tip_link'
    assert float(robot.find("gazebo[@reference='tool_tip_stop_joint']/springStiffness").text) > \
        float(robot.find("gazebo[@reference='tool_tip_joint']/springStiffness").text)
    assert float(robot.find("gazebo[@reference='tool_tip_joint']/springStiffness").text) > 0

    pinned = [j for j in robot.findall('joint') if j.find('parent').get('link') == 'world']
    assert [j.find('child').get('link') for j in pinned] == [profile.frames['world_base']]


def test_no_lift_profile_controls_only_the_arm():
    profile = prof.load_profile('ur10e_nolift')
    names = prof.controlled_joints(profile)
    assert None not in names and names == profile.arm_joints
    assert len(prof.home_positions(profile)) == len(names)


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


def _render_with(extra):
    return _render_with_hw(extra, 'gazebo')


def _render_with_hw(extra, hardware, profile_name='ur10e_telescopic'):
    profile = prof.load_profile(profile_name)
    doc = xacro.process_file(os.path.join(DESC, 'urdf', 'mobile_manipulator.urdf.xacro'),
                             mappings={**prof.xacro_mappings(profile, hardware), **extra})
    return ET.fromstring(doc.toxml())


def test_base_motion_and_depth_off_by_default():
    robot = _render_with({})
    assert robot.find("joint[@name='sim_world_joint']") is not None
    assert robot.find("link[@name='base_skid_link']") is None and robot.find("link[@name='depth_sensor_link']") is None
    assert not [p for p in robot.iter('plugin') if 'VelocityControl' in p.get('name', '')]
    assert all(s.get('type') != 'depth_camera' for s in robot.iter('sensor'))


def test_extra_plugin_params_only_when_asked():
    plugin = _render_with({}).find('gazebo/plugin')
    assert len(plugin.findall('parameters')) == 1
    plugin = _render_with({'gz_extra_params': '/tmp/gain.yaml'}).find('gazebo/plugin')
    assert [p.text for p in plugin.findall('parameters')][1:] == ['/tmp/gain.yaml']


def test_base_motion_frees_the_base():
    robot = _render_with({'gz_base_motion': 'true'})
    assert robot.find("joint[@name='sim_world_joint']") is None
    assert not [j for j in robot.findall('joint') if j.find('parent').get('link') == 'world']
    names = {p.get('name') for p in robot.iter('plugin')}
    assert {'gz::sim::systems::VelocityControl', 'gz::sim::systems::OdometryPublisher'} <= names
    odom = next(p for p in robot.iter('plugin') if (p.get('name') or '').endswith('OdometryPublisher'))
    assert odom.find('odom_frame').text == 'odom' and odom.find('robot_base_frame').text == 'base_footprint'
    assert robot.find("link[@name='base_skid_link']/collision") is not None


def test_depth_sensor_is_a_gz_depth_camera_on_depth_sensor_link():
    robot = _render_with({'gz_depth_sensor': 'true', 'gz_depth_xyz': '0.2 0 1.0', 'gz_depth_noise': '0.003'})
    j = robot.find("joint[@name='depth_sensor_joint']")
    assert j.find('parent').get('link') == 'base_footprint' and j.find('origin').get('xyz') == '0.2 0 1.0'
    s = robot.find("gazebo[@reference='depth_sensor_link']/sensor[@type='depth_camera']")
    assert s.find('topic').text == '/depth_sensor' and s.find('gz_frame_id').text == 'depth_sensor_link'
    assert float(s.find('camera/noise/stddev').text) == 0.003


def test_lidar_off_by_default_and_a_gz_gpu_lidar_on_lidar_link():
    assert _render_with({}).find("link[@name='lidar_link']") is None
    robot = _render_with({'gz_lidar': 'true', 'gz_lidar_xyz': '0.4 0 0.3', 'gz_lidar_samples': '181',
                          'gz_lidar_noise': '0.01'})
    j = robot.find("joint[@name='lidar_joint']")
    assert j.find('parent').get('link') == 'base_footprint' and j.find('origin').get('xyz') == '0.4 0 0.3'
    s = robot.find("gazebo[@reference='lidar_link']/sensor[@type='gpu_lidar']")
    assert s.find('topic').text == '/scan' and s.find('gz_frame_id').text == 'lidar_link'
    assert int(s.find('lidar/scan/horizontal/samples').text) == 181
    assert float(s.find('lidar/noise/stddev').text) == 0.01
    assert float(s.find('lidar/range/max').text) == 10.0


def test_imu_off_by_default_and_a_gz_imu_with_opt_in_noise_on_imu_link():
    assert _render_with({}).find("link[@name='imu_link']") is None
    robot = _render_with({'gz_imu': 'true', 'gz_imu_xyz': '0 0 0.2', 'gz_imu_gyro_noise': '0.002',
                          'gz_imu_accel_bias': '0.05'})
    j = robot.find("joint[@name='imu_joint']")
    assert j.find('parent').get('link') == 'base_footprint' and j.find('origin').get('xyz') == '0 0 0.2'
    s = robot.find("gazebo[@reference='imu_link']/sensor[@type='imu']")
    assert s.find('topic').text == '/imu' and s.find('gz_frame_id').text == 'imu_link'
    assert float(s.find('imu/angular_velocity/z/noise/stddev').text) == 0.002
    assert float(s.find('imu/angular_velocity/z/noise/bias_stddev').text) == 0.0
    assert float(s.find('imu/linear_acceleration/x/noise/bias_stddev').text) == 0.05


def test_rgb_camera_off_by_default_and_a_gz_camera_on_rgb_camera_link():
    assert _render_with({}).find("link[@name='rgb_camera_link']") is None
    robot = _render_with({'gz_rgb_camera': 'true', 'gz_rgb_xyz': '0.2 0 1.0', 'gz_rgb_width': '640', 'gz_rgb_height': '480'})
    j = robot.find("joint[@name='rgb_camera_joint']")
    assert j.find('parent').get('link') == 'base_footprint' and j.find('origin').get('xyz') == '0.2 0 1.0'
    s = robot.find("gazebo[@reference='rgb_camera_link']/sensor[@type='camera']")
    assert s.find('topic').text == '/rgb_camera' and s.find('gz_frame_id').text == 'rgb_camera_link'
    assert s.find('camera/image/width').text == '640' and s.find('camera/image/height').text == '480'


def _urdf_mimics(robot):
    return {j.get('name'): j.find('mimic') for j in robot.findall('joint') if j.find('mimic') is not None}


@pytest.mark.parametrize('stages', [0, 1, 2, 3])
def test_gazebo_lift_stage_mimic(stages):
    """gz-sim ignores the URDF <mimic>: every stage joint needs the mimic params in ros2_control."""
    robot = _render_with_hw({'lift': 'telescopic', 'lift_stages': str(stages), 'lift_stroke': '1.2'}, 'gazebo', 'generic_6dof_telescopic')
    rc = robot.find("ros2_control[@name='mobile_manipulator_system']")
    declared = {j.get('name'): j for j in rc.findall('joint') if j.find("param[@name='mimic']") is not None}
    urdf = {n: m for n, m in _urdf_mimics(robot).items() if n.startswith('lift_stage_')}
    assert sorted(declared) == sorted(urdf) == [f'lift_stage_{i}_joint' for i in range(1, stages + 1)]
    for name, joint in declared.items():
        params = {p.get('name'): p.text for p in joint.findall('param')}
        assert params['mimic'] == urdf[name].get('joint') == 'lift_joint'
        assert float(params['multiplier']) == pytest.approx(float(urdf[name].get('multiplier')))
        assert float(params['multiplier']) == pytest.approx(1.0 / stages)
        assert float(params['offset']) == pytest.approx(float(urdf[name].get('offset')))
        assert joint.find('command_interface') is None  # never commanded
        assert {s.get('name') for s in joint.findall('state_interface')} >= {'position', 'velocity'}


@pytest.mark.parametrize('hardware', ['mock', 'jtac_mock'])
def test_stage_mimic_params_only_in_gazebo(hardware):
    robot = _render_with_hw({'lift_stages': '2'}, hardware, 'generic_6dof_telescopic')
    assert robot.find(".//ros2_control//param[@name='mimic']") is None


def test_gazebo_pinned_lift_declares_no_stage_joints():
    robot = _render_with_hw({'lift_fixed_height': '0.3'}, 'gazebo', 'generic_6dof_telescopic')
    assert robot.find(".//ros2_control//param[@name='mimic']") is None
