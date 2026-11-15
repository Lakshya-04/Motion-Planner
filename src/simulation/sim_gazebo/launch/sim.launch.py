"""Gazebo Harmonic simulation of the mobile manipulator.

  ros2 launch sim_gazebo sim.launch.py                                   # headless, joint_trajectory_controller
  ros2 launch sim_gazebo sim.launch.py gui:=true                         # gz GUI (needs a display)
  ros2 launch sim_gazebo sim.launch.py controller:=jtac                  # compliant JTAC controller
  ros2 launch sim_gazebo sim.launch.py wall_scene:=/path/wall_scene.json wall_id:=0 standoff:=0.9
  ros2 launch sim_gazebo sim.launch.py profile:=ur10e_telescopic
  ros2 launch sim_gazebo sim.launch.py base_motion:=true                 # planar base on /cmd_vel, /odom, TF odom -> base_footprint
  ros2 launch sim_gazebo sim.launch.py depth_sensor:=true                # gz depth camera -> /depth_sensor/points

What starts: gz sim (server only unless gui:=true), robot_state_publisher, the robot spawned from
the description rendered with hardware:=gazebo from the profile's xacro args, ros_gz_bridge for
/clock, the raw wrench (/ft_sensor/wrench_gz) and the ground-truth contacts,
plus (base_motion:=true) /cmd_vel, /odom and TF odom -> base_footprint through gz-sim's VelocityControl and
OdometryPublisher systems, plus (depth_sensor:=true) /depth_sensor/points from a gz depth_camera (the world gets the
Sensors system, ogre2 renders headless in software),
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
SENSORS_PLUGIN = ('  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
                  '<render_engine>ogre2</render_engine></plugin>\n')
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


def _jtac_params(profile, joints, robot_description, conditioned=False, ft_filter=1.0, position_only=False):
    """The existing JTAC config with names/frames taken from the profile instead of the demo's."""
    cfg_path = os.path.join(get_package_share_directory('compliant_trajectory_controller'),
                            'config', 'jtac_controller.yaml')
    with open(cfg_path) as f:
        params = yaml.safe_load(f)['combined_arm_lift_controller']['ros__parameters']
    fr = prof.frames(profile)
    params['joints'] = joints
    if position_only:
        # Position only. With the velocity command interface claimed too, gz_ros2_control drives the joints open loop
        # from the velocity (the trajectory feed-forward) and ignores the position command, so a moving reference is
        # tracked far behind (a 0.01 m/s stroke fell 0.08 m short, coverage 0.88). The position command then goes
        # through gz_ros2_control's own P loop (gain 0.1/s by default: slow, soft; see docs/sim_gazebo.md).
        params['command_interfaces'] = ['position']
    params['robot_description'] = robot_description
    params['kinematics'].update({'base': fr['base'], 'tip': fr['tcp']})
    if conditioned:
        # contact wrench at the tcp from ft_conditioner through the ft_bridge_hardware sensor
        params['ft_sensor'] = {**params.get('ft_sensor', {}), 'name': 'ft_sensor_tcp',
                               'frame': {'id': fr['tcp']}}
    else:
        params['ft_sensor'] = {**params.get('ft_sensor', {}), 'name': 'ft_sensor',
                               'frame': {'id': FT_FRAME}}
    params['ft_sensor']['filter_coefficient'] = ft_filter
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
    base_motion = lc('base_motion') == 'true'
    depth = lc('depth_sensor') == 'true'
    mappings = prof.xacro_mappings(profile, 'gazebo')
    if base_motion:
        mappings['gz_base_motion'] = 'true'
    if depth:
        pose = [float(v) for v in lc('depth_sensor_pose').split()]
        if len(pose) != 6:
            raise ValueError('depth_sensor_pose needs "x y z roll pitch yaw"')
        mappings.update(gz_depth_sensor='true', gz_depth_xyz=' '.join(map(str, pose[:3])),
                        gz_depth_rpy=' '.join(map(str, pose[3:])), gz_depth_noise=lc('depth_sensor_noise_m'),
                        gz_depth_rate=lc('depth_sensor_rate_hz'))
    robot_description = xacro.process_file(
        os.path.join(desc_pkg, 'urdf', 'mobile_manipulator.urdf.xacro'), mappings=mappings).toxml()

    world = lc('world') or os.path.join(pkg, 'worlds', 'empty.sdf')
    if lc('wall_scene'):
        world = os.path.join(tempfile.mkdtemp(prefix='sim_gazebo_world_'), 'wall.sdf')
        wall_world.generate_world(lc('wall_scene'), world, wall_id=int(lc('wall_id')),
                                  standoff=float(lc('standoff')))
    if depth:
        # the render sensors need the Sensors system in the world (ogre2; no display: gz falls back to headless)
        with open(world) as f:
            sdf = f.read()
        if 'gz-sim-sensors-system' not in sdf:
            sdf = sdf.replace('</world>', SENSORS_PLUGIN + '</world>', 1)
            world = os.path.join(tempfile.mkdtemp(prefix='sim_gazebo_world_'), 'world_sensors.sdf')
            with open(world, 'w') as f:
                f.write(sdf)

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
    bridged = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
               '/ft_sensor/wrench@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
               '/tool_contact/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts']
    remaps = [('/ft_sensor/wrench', '/ft_sensor/wrench_gz')]
    if base_motion:
        # gz-side topics /base/* (VelocityControl, OdometryPublisher) -> the ROS names the MuJoCo sim uses
        bridged += ['/base/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    '/base/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    '/base/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V']
        remaps += [('/base/cmd_vel', '/cmd_vel'), ('/base/odom', '/odom'), ('/base/tf', '/tf')]
    if depth:
        bridged.append('/depth_sensor/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked')
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  arguments=bridged, remappings=remaps,
                  parameters=[{'use_sim_time': True}])
    truth = Node(package='sim_gazebo', executable='contact_ground_truth.py', output='screen',
                 parameters=[{'use_sim_time': True, 'tcp_frame': fr['tcp']}])

    actions = [gz, rsp, bridge, truth, spawn]
    conditioned = lc('ft_conditioner') == 'true'
    if conditioned:
        # raw wrench (ft_sensor_link, with the payload weight) -> contact wrench at the tcp. The
        # payload is not set: calibrate it with ~/capture_pose + ~/identify (or pass payload_mass_kg).
        actions.append(Node(
            package='ft_conditioner', executable='wrench_conditioner_node', name='ft_conditioner',
            output='screen',
            parameters=[{'use_sim_time': True,
                         'input_topic': '/force_torque_sensor_broadcaster/wrench',
                         'output_topic': '/ft_conditioner/wrench_conditioned',
                         'world_frame': fr['base'], 'tool_frame': fr['tcp'],
                         'payload_mass_kg': float(lc('payload_mass_kg'))}]))
    if lc('controller') != 'none':
        name = CONTROLLER_NAMES[lc('controller')]
        params = (_jtc_params(joints) if lc('controller') == 'jtc'
                  else _jtac_params(profile, joints, robot_description, conditioned,
                                    float(lc('ft_filter')), lc('jtac_position_only') == 'true'))
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
    if lc('move_group') == 'true':
        # MoveIt for planning only (the same description the sim runs, hardware jtac_mock); execution stays
        # with the controller started above
        from mobile_manipulator_moveit_config import build_moveit_config
        moveit_config = build_moveit_config(profile=lc('profile'), hardware='jtac_mock')
        actions.append(Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                            parameters=[moveit_config.to_dict(), {'use_sim_time': True}]))
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
        DeclareLaunchArgument('ft_conditioner', default_value='false',
                              description='true: start ft_conditioner (raw wrench -> tcp contact wrench) and '
                                          'point JTAC at the ft_conditioned_bridge sensor ft_sensor_tcp'),
        DeclareLaunchArgument('ft_filter', default_value='1.0',
                              description='JTAC ft_sensor.filter_coefficient (1.0: no low-pass; smaller: smoother)'),
        DeclareLaunchArgument('payload_mass_kg', default_value='0.0',
                              description='initial payload mass for ft_conditioner (0: calibrate with ~/identify)'),
        DeclareLaunchArgument('jtac_position_only', default_value='false',
                              description='true: JTAC claims only the position command interface (closed gz position '
                                          'loop, slow and soft) instead of position + velocity (open loop on the '
                                          'velocity); the manager pass uses true'),
        DeclareLaunchArgument('move_group', default_value='false',
                              description='also start move_group (planning only) for the manipulation manager'),
        DeclareLaunchArgument('base_motion', default_value='false',
                              description='true: free planar base driven by geometry_msgs/Twist on /cmd_vel (gz-sim '
                                          'VelocityControl, omni; the profile drive_type is not applied), nav_msgs/'
                                          'Odometry on /odom and TF odom -> base_footprint (OdometryPublisher); false '
                                          'pins the base to the world'),
        DeclareLaunchArgument('depth_sensor', default_value='false',
                              description='true: gz depth_camera on base_footprint, PointCloud2 on /depth_sensor/points '
                                          'in frame depth_sensor_link (needs gz rendering, ogre2); false adds nothing'),
        DeclareLaunchArgument('depth_sensor_pose', default_value='0.1 0 1.15 0 0 0',
                              description='"x y z roll pitch yaw" of depth_sensor_link in base_footprint (x forward)'),
        DeclareLaunchArgument('depth_sensor_noise_m', default_value='0.0', description='Gaussian range noise std, m'),
        DeclareLaunchArgument('depth_sensor_rate_hz', default_value='5.0', description='depth sensor rate, Hz'),
        OpaqueFunction(function=_launch),
    ])
