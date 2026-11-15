"""Gazebo Harmonic simulation of the mobile manipulator.

  ros2 launch sim_gazebo sim.launch.py                                   # headless, joint_trajectory_controller
  ros2 launch sim_gazebo sim.launch.py gui:=true                         # gz GUI (needs a display)
  ros2 launch sim_gazebo sim.launch.py controller:=jtac                  # compliant JTAC controller
  ros2 launch sim_gazebo sim.launch.py wall_scene:=/path/wall_scene.json wall_id:=0 standoff:=0.9
  ros2 launch sim_gazebo sim.launch.py profile:=ur10e_telescopic

What starts: gz sim (server only unless gui:=true), robot_state_publisher, the robot spawned from
the description rendered with hardware:=gazebo from the profile's xacro args, ros_gz_bridge for
/clock, the raw wrench (/ft_sensor/wrench_gz) and the ground-truth contacts,
plus the broadcaster topic /force_torque_sensor_broadcaster/wrench (frame ft_sensor_link), and the controller spawners. The controller_manager
lives inside the gz server (gz_ros2_control), not in a separate node.
"""
import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sim_gazebo import profile as prof
from sim_gazebo import wall_world

FT_FRAME = 'ft_sensor_link'   # raw wrench frame: the sensor body between flange and tool
CONTROLLER_NAMES = {'jtc': 'joint_trajectory_controller', 'jtac': 'combined_arm_lift_controller'}


def _write_yaml(data, prefix):
    fd, path = tempfile.mkstemp(prefix=prefix, suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(data, f)
    return path


def _jtc_params(joints):
    return {'joint_trajectory_controller': {'ros__parameters': {
        'joints': joints,
        'command_interfaces': ['position'],
        'state_interfaces': ['position', 'velocity'],
        'allow_nonzero_velocity_at_trajectory_end': True,
        'constraints': {'stopped_velocity_tolerance': 0.05, 'goal_time': 2.0}}}}


def _jtac_params(profile, joints, robot_description):
    """The existing JTAC config with names/frames taken from the profile instead of the demo's."""
    cfg_path = os.path.join(get_package_share_directory('compliant_trajectory_controller'),
                            'config', 'jtac_controller.yaml')
    with open(cfg_path) as f:
        params = yaml.safe_load(f)['combined_arm_lift_controller']['ros__parameters']
    fr = prof.frames(profile)
    params['joints'] = joints
    params['robot_description'] = robot_description
    params['kinematics'].update({'base': fr['base'], 'tip': fr['tcp']})
    params['ft_sensor'] = {**params.get('ft_sensor', {}), 'name': 'ft_sensor',
                           'frame': {'id': FT_FRAME}}
    params['control'] = {'frame': {'id': fr['tcp']}}
    params['fixed_world_frame'] = {'frame': {'id': fr['base']}}
    params.setdefault('gravity_compensation', {})['frame'] = {'id': fr['tcp']}
    return {'combined_arm_lift_controller': {'ros__parameters': params}}


def _spawner(name, param_file=None, extra=()):
    args = [name, '--controller-manager', '/controller_manager', '--controller-manager-timeout', '60']
    if param_file:
        args += ['--param-file', param_file]
    return Node(package='controller_manager', executable='spawner', arguments=args + list(extra),
                output='screen')


def _launch(context):
    pkg = get_package_share_directory('sim_gazebo')
    lc = lambda n: context.perform_substitution(LaunchConfiguration(n))  # noqa: E731
    profile = prof.load_profile(lc('profile'))
    joints = prof.controlled_joints(profile)
    fr = prof.frames(profile)

    desc_pkg = get_package_share_directory('mobile_manipulator_description')
    robot_description = xacro.process_file(
        os.path.join(desc_pkg, 'urdf', 'mobile_manipulator.urdf.xacro'),
        mappings=prof.xacro_mappings(profile, 'gazebo')).toxml()

    world = lc('world') or os.path.join(pkg, 'worlds', 'empty.sdf')
    if lc('wall_scene'):
        world = os.path.join(tempfile.mkdtemp(prefix='sim_gazebo_world_'), 'wall.sdf')
        wall_world.generate_world(lc('wall_scene'), world, wall_id=int(lc('wall_id')),
                                  standoff=float(lc('standoff')))

    gz_args = f'-r -v2 {world}' + ('' if lc('gui') == 'true' else ' -s')
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
               output='screen')
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', profile.robot_name,
                            '-z', '0.0'])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                             '/ft_sensor/wrench@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
                             '/tool_contact/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'],
                  remappings=[('/ft_sensor/wrench', '/ft_sensor/wrench_gz')],
                  parameters=[{'use_sim_time': True}])
    truth = Node(package='sim_gazebo', executable='contact_ground_truth.py', output='screen',
                 parameters=[{'use_sim_time': True, 'tcp_frame': fr['tcp']}])

    actions = [gz, rsp, bridge, truth, spawn]
    if lc('controller') != 'none':
        name = CONTROLLER_NAMES[lc('controller')]
        params = (_jtc_params(joints) if lc('controller') == 'jtc'
                  else _jtac_params(profile, joints, robot_description))
        jsb = _spawner('joint_state_broadcaster')
        ftb = _spawner('force_torque_sensor_broadcaster', _write_yaml(
            {'force_torque_sensor_broadcaster': {'ros__parameters': {
                'sensor_name': 'ft_sensor', 'frame_id': FT_FRAME}}}, 'sim_gz_ftb_'))
        ctl = _spawner(name, _write_yaml(params, 'sim_gz_ctl_'))
        actions += [
            RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
            RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[ftb])),
            RegisterEventHandler(OnProcessExit(target_action=ftb, on_exit=[ctl])),
        ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='generic_6dof_telescopic',
                              description='description profile name or path (config/profiles)'),
        DeclareLaunchArgument('world', default_value='', description='SDF world (default: empty ground)'),
        DeclareLaunchArgument('wall_scene', default_value='',
                              description='wall_scene.json / SceneModel YAML; generates the world'),
        DeclareLaunchArgument('wall_id', default_value='0'),
        DeclareLaunchArgument('standoff', default_value='0.9', description='wall distance in front of the base (m)'),
        DeclareLaunchArgument('gui', default_value='false', description='true opens the gz GUI (server only otherwise)'),
        DeclareLaunchArgument('controller', default_value='jtc', description='jtc | jtac | none'),
        OpaqueFunction(function=_launch),
    ])
