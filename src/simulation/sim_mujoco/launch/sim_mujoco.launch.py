"""Headless MuJoCo simulation of the mobile manipulator under ros2_control (hardware:=mujoco).

Steps: render the robot URDF from the profile, generate the MJCF (plus the wall world when `scene`
is given), render robot_description with hardware:=mujoco, then start controller_manager (with the
MuJoCo plugin), robot_state_publisher, joint_state_broadcaster and one trajectory controller named
`combined_arm_lift_controller` (the name MoveIt's controller config already uses).

    ros2 launch sim_mujoco sim_mujoco.launch.py profile:=ur10e_telescopic
    ros2 launch sim_mujoco sim_mujoco.launch.py scene:=/path/wall_scene.json scene_offset:="0.6 0 0 0"
    ros2 launch sim_mujoco sim_mujoco.launch.py controller:=jtac      # compliant controller
    ros2 launch sim_mujoco sim_mujoco.launch.py viewer:=true          # optional, needs a display
    ros2 launch sim_mujoco sim_mujoco.launch.py controller:=jtac conditioner:=true   # contact path

controller:=jtc  joint_trajectory_controller (position command; default)
controller:=jtac compliant_trajectory_controller/JtacController from the repo's own
                 jtac_controller.yaml, with the joint names, kinematics chain, F/T sensor name and
                 frames overridden for this robot; it reads `ft_sensor` from this plugin. That is the
                 raw wrench at tool_mount_link, so the sensor frame is the mount and the tool weight
                 is compensated through JTAC's gravity_compensation (CoG from the URDF). The lever
                 arm to the TCP and the bias are not removed: engage force axes only. With
                 conditioner:=true it reads `ft_sensor_tcp` (the conditioned contact wrench at the
                 TCP) instead, with the sensor frame at the TCP and no gravity compensation, because
                 the conditioner already removed the tool weight.

base_motion:=true adds a planar base (three joints under base_footprint, stiff position hold): send
                 geometry_msgs/Twist on /cmd_vel (body frame; vy ignored for diff / skid profiles,
                 limited to the profile's max velocities, zero after 0.5 s without a command); the plugin
                 publishes nav_msgs/Odometry on /odom and TF odom -> base_footprint. Wheels stay visual.

conditioner:=true also starts the ft_conditioner node (raw /sim_mujoco/ft_raw -> contact wrench at
                 tcp_link on /ft_conditioner/wrench_conditioned, which the description's second
                 ros2_control block `ft_conditioned_bridge` exports as the sensor `ft_sensor_tcp`).
                 It starts uncalibrated (payload 0): call /ft_conditioner/capture_pose in several
                 poses and /ft_conditioner/identify, or pass payload_mass_kg / payload_cog.
"""
import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from sim_mujoco.generate import (command_joints, generate, jtac_overrides, load_profile, render_urdf,
                                 tool_payload)

CONTROLLER_NAME = 'combined_arm_lift_controller'
FT_SENSOR = 'ft_sensor'
FT_SENSOR_CONDITIONED = 'ft_sensor_tcp'


def _jtc_params(joints, update_rate):
    return {
        'controller_manager': {'ros__parameters': {
            'update_rate': update_rate,
            CONTROLLER_NAME: {'type': 'joint_trajectory_controller/JointTrajectoryController'},
            'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
        }},
        CONTROLLER_NAME: {'ros__parameters': {
            'joints': joints,
            'command_interfaces': ['position'],
            'state_interfaces': ['position', 'velocity'],
            'allow_nonzero_velocity_at_trajectory_end': True,
            'constraints': {'stopped_velocity_tolerance': 0.05, 'goal_time': 2.0},
        }},
    }


def _jtac_params(joints, update_rate, robot_description, profile, ft_sensor=FT_SENSOR,
                 conditioned=False):
    base = yaml.safe_load(open(os.path.join(
        get_package_share_directory('compliant_trajectory_controller'), 'config',
        'jtac_controller.yaml')))
    base['controller_manager']['ros__parameters']['update_rate'] = update_rate
    frames = profile.get('frames', {})
    mass, cog = tool_payload(robot_description, frames.get('tool_mount', 'tool_mount_link'))
    if conditioned:
        # The conditioner already moved the wrench to the TCP and removed the tool weight.
        frames = dict(frames, tool_mount=frames.get('tcp', 'tcp_link'))
        mass, cog = 0.0, (0.0, 0.0, 0.0)
    return jtac_overrides(base, joints, robot_description, frames, ft_sensor, mass, cog)


def _setup(context, *args, **kwargs):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    profile = load_profile(arg('profile'))
    joints = command_joints(profile)
    out_dir = arg('out_dir') or tempfile.mkdtemp(prefix='sim_mujoco_')
    scene = arg('scene') or None
    offset = [float(v) for v in arg('scene_offset').split()]

    mjcf, skipped = generate(profile, render_urdf(profile), out_dir, scene=scene,
                             scene_offset=tuple(offset),
                             base_motion=arg('base_motion') == 'true')
    for s in skipped:
        print(f'[sim_mujoco] skipped scene object {s}')
    print(f'[sim_mujoco] MJCF: {mjcf}')

    os.environ['SIM_MUJOCO_MJCF'] = str(mjcf)
    xacro_file = os.path.join(get_package_share_directory('mobile_manipulator_description'),
                              'urdf', 'mobile_manipulator.urdf.xacro')
    mappings = {k: str(v) for k, v in profile['robot'].get('xacro_args', {}).items()}
    mappings['hardware'] = 'mujoco'
    description = xacro.process_file(xacro_file, mappings=mappings).toxml()
    robot_description = {'robot_description': ParameterValue(description, value_type=str)}

    update_rate = int(arg('update_rate'))
    jtac = arg('controller') == 'jtac'
    conditioner = arg('conditioner') == 'true'
    params = (_jtac_params(joints, update_rate, description, profile,
                           FT_SENSOR_CONDITIONED if conditioner else FT_SENSOR,
                           conditioned=conditioner) if jtac
              else _jtc_params(joints, update_rate))
    fd, params_file = tempfile.mkstemp(prefix='sim_mujoco_controllers_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(params, f)
    # robot_description must also be a parameter of the JTAC node itself, see jtac_controller.yaml
    controller_args = ['--param-file', params_file] if jtac else []

    control_node = Node(package='controller_manager', executable='ros2_control_node',
                        parameters=[robot_description, params_file], output='screen')
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[robot_description], output='screen')
    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
               output='screen')
    controller = Node(package='controller_manager', executable='spawner',
                      arguments=[CONTROLLER_NAME, '--controller-manager', '/controller_manager',
                                 *controller_args],
                      output='screen')
    viewer = Node(package='sim_mujoco', executable='viewer.py',
                  arguments=[str(mjcf), *joints], condition=IfCondition(LaunchConfiguration('viewer')),
                  output='screen')
    nodes = [control_node, rsp, jsb,
             RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[controller])), viewer]
    if arg('base_motion') == 'true':
        # the plugin publishes the base pose as geometry_msgs only; this turns it into /odom and TF
        nodes.append(Node(package='sim_mujoco', executable='base_odom.py', name='base_odom', output='screen',
                          parameters=[{'base_frame': (profile.get('frames', {}).get('world_base')
                                                      or 'base_footprint')}]))
    if conditioner:
        # raw wrench is in tool_mount_link (the sensor frame) and includes the payload weight; sign
        # +1 (the plugin reports the load the tool puts on the sensor); the sensor-to-tcp offset and
        # the sensor's orientation come from TF (robot_state_publisher). base_link is z-up (welded).
        nodes.append(Node(
            package='ft_conditioner', executable='wrench_conditioner_node', name='ft_conditioner',
            output='screen', parameters=[{
                'input_topic': '/sim_mujoco/ft_raw',
                'output_topic': '/ft_conditioner/wrench_conditioned',
                'world_frame': 'base_link',
                'tool_frame': 'tcp_link',
                'sign': 1.0,
                'payload_mass_kg': float(arg('payload_mass_kg')),
                'payload_cog_xyz': [float(v) for v in arg('payload_cog').split()],
            }]))
    if arg('move_group') == 'true':
        # MoveIt for planning only (compute_cartesian_path): the same description the sim runs, execution
        # stays with the controller started above.
        from mobile_manipulator_moveit_config import build_moveit_config
        moveit_config = build_moveit_config(profile=arg('profile'), hardware='jtac_mock')
        nodes.append(Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                          parameters=[moveit_config.to_dict()]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='ur10e_telescopic',
                              description='profile name or YAML path (config/profiles)'),
        DeclareLaunchArgument('scene', default_value='',
                              description='wall_scene.json / scene_model scene to add as a world'),
        DeclareLaunchArgument('scene_offset', default_value='0 0 0 0',
                              description='scene pose in the robot base frame: "x y z yaw"'),
        DeclareLaunchArgument('controller', default_value='jtc', description='jtc or jtac'),
        DeclareLaunchArgument('conditioner', default_value='false',
                              description='start ft_conditioner; with controller:=jtac JTAC then reads '
                                          'the conditioned wrench (sensor ft_sensor_tcp)'),
        DeclareLaunchArgument('payload_mass_kg', default_value='0.0',
                              description='conditioner payload mass; 0 = uncalibrated, use the '
                                          'capture_pose / identify services'),
        DeclareLaunchArgument('payload_cog', default_value='0 0 0',
                              description='conditioner payload centre of gravity in the sensor frame'),
        DeclareLaunchArgument('move_group', default_value='false',
                              description='also start move_group (planning only) for scripted cartesian passes'),
        DeclareLaunchArgument('base_motion', default_value='false',
                              description='planar base (x, y, yaw) driven by geometry_msgs/Twist on /cmd_vel '
                                          '(omni or diff per the profile drive_type), odom on /odom and TF '
                                          'odom -> base_footprint; false keeps the base static'),
        DeclareLaunchArgument('update_rate', default_value='100', description='control rate, Hz'),
        DeclareLaunchArgument('out_dir', default_value='', description='where to write the MJCF'),
        DeclareLaunchArgument('viewer', default_value='false',
                              description='open the MuJoCo viewer (needs a display and python mujoco)'),
        OpaqueFunction(function=_setup),
    ])
