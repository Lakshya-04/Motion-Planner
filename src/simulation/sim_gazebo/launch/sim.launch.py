"""Gazebo Harmonic simulation of the mobile manipulator.

  ros2 launch sim_gazebo sim.launch.py                                   # headless, joint_trajectory_controller
  ros2 launch sim_gazebo sim.launch.py gui:=true                         # gz GUI (needs a display)
  ros2 launch sim_gazebo sim.launch.py controller:=jtac                  # compliant JTAC controller
  ros2 launch sim_gazebo sim.launch.py wall_scene:=/path/wall_scene.json wall_id:=0 standoff:=0.9
  ros2 launch sim_gazebo sim.launch.py profile:=ur10e_telescopic
  ros2 launch sim_gazebo sim.launch.py base_motion:=true                 # planar base on /cmd_vel, /odom, TF odom -> base_footprint
  ros2 launch sim_gazebo sim.launch.py rgb_camera:=true                  # gz colour camera -> /rgb_camera/image
  ros2 launch sim_gazebo sim.launch.py depth_sensor:=true                # gz depth camera -> /depth_sensor/points
  ros2 launch sim_gazebo sim.launch.py lidar:=true                       # gz gpu_lidar -> /scan (LaserScan)

What starts: gz sim (server only unless gui:=true), robot_state_publisher, the robot spawned from
the description rendered with hardware:=gazebo from the profile's xacro args, ros_gz_bridge for
/clock, the raw wrench (/ft_sensor/wrench_gz) and the ground-truth contacts,
plus (base_motion:=true) /cmd_vel, /odom and TF odom -> base_footprint through gz-sim's VelocityControl and
OdometryPublisher systems, plus (depth_sensor:=true) /depth_sensor/points from a gz depth_camera (the world gets the
Sensors system, ogre2 renders headless in software),
plus the broadcaster topic /force_torque_sensor_broadcaster/wrench (frame ft_sensor_link), and the controller spawners. The controller_manager
lives inside the gz server (gz_ros2_control), not in a separate node.
"""
import json
import math
import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sim_gazebo import profile as prof
from sim_gazebo import wall_world

FT_FRAME = 'ft_sensor_link'   # raw wrench frame: the sensor body between flange and tool
SENSORS_PLUGIN = ('  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
                  '<render_engine>ogre2</render_engine></plugin>\n')
WORLD_CAMERA_MODEL = '''  <model name="rgb_camera_world">
    <static>true</static>
    <pose>{pose}</pose>
    <link name="rgb_camera_world_link">
      <sensor name="rgb_camera" type="camera">
        <always_on>true</always_on>
        <update_rate>{rate}</update_rate>
        <visualize>false</visualize>
        <topic>/rgb_camera</topic>
        <camera>
          <horizontal_fov>1.0472</horizontal_fov>
          <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>20.0</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
'''
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
    if lc('position_gain'):
        # the plugin's own ROS node is called gz_ros_control
        mappings['gz_extra_params'] = _write_yaml(
            {'gz_ros_control': {'ros__parameters': {'position_proportional_gain': float(lc('position_gain'))}}},
            'sim_gz_gain_')
    if base_motion:
        mappings['gz_base_motion'] = 'true'
    if depth:
        pose = [float(v) for v in lc('depth_sensor_pose').split()]
        if len(pose) != 6:
            raise ValueError('depth_sensor_pose needs "x y z roll pitch yaw"')
        mappings.update(gz_depth_sensor='true', gz_depth_xyz=' '.join(map(str, pose[:3])),
                        gz_depth_rpy=' '.join(map(str, pose[3:])), gz_depth_noise=lc('depth_sensor_noise_m'),
                        gz_depth_rate=lc('depth_sensor_rate_hz'))
    lidar = lc('lidar') == 'true'
    if lidar:
        pose = [float(v) for v in lc('lidar_pose').split()]
        if len(pose) != 6:
            raise ValueError('lidar_pose needs "x y z roll pitch yaw"')
        lo, hi = math.radians(float(lc('lidar_angle_min_deg'))), math.radians(float(lc('lidar_angle_max_deg')))
        step = math.radians(float(lc('lidar_angle_increment_deg')))
        mappings.update(gz_lidar='true', gz_lidar_xyz=' '.join(map(str, pose[:3])),
                        gz_lidar_rpy=' '.join(map(str, pose[3:])), gz_lidar_min_angle=repr(lo),
                        gz_lidar_max_angle=repr(hi), gz_lidar_samples=str(int((hi - lo) / step + 1e-3) + 1),
                        gz_lidar_min=lc('lidar_range_min_m'), gz_lidar_max=lc('lidar_range_max_m'),
                        gz_lidar_rate=lc('lidar_rate_hz'), gz_lidar_noise=lc('lidar_noise_m'))
    rgb = lc('rgb_camera') == 'true'
    world_camera = ''
    if rgb:
        pose = [float(v) for v in lc('rgb_camera_pose').split()]
        size = lc('rgb_camera_size').split()
        if len(pose) != 6 or len(size) != 2:
            raise ValueError('rgb_camera_pose needs "x y z roll pitch yaw", rgb_camera_size "width height"')
        if lc('rgb_camera_frame') not in ('base', 'world'):
            raise ValueError('rgb_camera_frame is base or world')
    if rgb and lc('rgb_camera_frame') == 'world':
        # a static model in the world, so the camera stays put when the base drives (multi-stand films)
        world_camera = WORLD_CAMERA_MODEL.format(pose=' '.join(map(str, pose)), w=size[0], h=size[1],
                                                 rate=lc('rgb_camera_rate_hz'))
    elif rgb:
        mappings.update(gz_rgb_camera='true', gz_rgb_xyz=' '.join(map(str, pose[:3])),
                        gz_rgb_rpy=' '.join(map(str, pose[3:])), gz_rgb_width=size[0], gz_rgb_height=size[1],
                        gz_rgb_rate=lc('rgb_camera_rate_hz'))
    robot_description = xacro.process_file(
        os.path.join(desc_pkg, 'urdf', 'mobile_manipulator.urdf.xacro'), mappings=mappings).toxml()

    world = lc('world') or os.path.join(pkg, 'worlds', 'empty.sdf')
    rviz_world_wall = ''
    if lc('wall_scene'):
        world = os.path.join(tempfile.mkdtemp(prefix='sim_gazebo_world_'), 'wall.sdf')
        shapes = wall_world.generate_world(lc('wall_scene'), world, wall_id=int(lc('wall_id')),
                                           standoff=float(lc('standoff')))
        if os.path.splitext(lc('wall_scene'))[1].lower() == '.json':
            # RViz must draw the wall where the world put it (the scene file is in the as-mapped frame: plane x = 2 for the
            # smoke wall, the world's is `standoff` in front of the base), so write the placed wall for mp_viz_scene
            rviz_world_wall = os.path.join(os.path.dirname(world), 'rviz_world_wall.json')
            with open(rviz_world_wall, 'w') as f:
                json.dump(wall_world.world_wall_scene(shapes, int(lc('wall_id'))), f, indent=2)
    if depth or rgb or lidar:
        # the render sensors need the Sensors system in the world (ogre2; no display: gz falls back to headless)
        with open(world) as f:
            sdf = f.read()
        if 'gz-sim-sensors-system' not in sdf or world_camera:
            if 'gz-sim-sensors-system' not in sdf:
                sdf = sdf.replace('</world>', SENSORS_PLUGIN + '</world>', 1)
            sdf = sdf.replace('</world>', world_camera + '</world>', 1)
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
        # odom_realism on: the gz odometry becomes /odom_truth and its TF stays off /tf (sim_realism publishes both)
        odom_real = lc('odom_realism') != 'off'
        remaps += [('/base/cmd_vel', '/cmd_vel'), ('/base/odom', '/odom_truth' if odom_real else '/odom'),
                   ('/base/tf', '/base/tf_truth' if odom_real else '/tf')]
    if rgb:
        bridged.append('/rgb_camera@sensor_msgs/msg/Image[gz.msgs.Image')
        remaps.append(('/rgb_camera', '/rgb_camera/image'))
    if depth:
        bridged.append('/depth_sensor/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked')
        if lc('depth_realism') != 'off':
            remaps.append(('/depth_sensor/points', '/depth_sensor/points_truth'))
    if lidar:
        bridged.append('/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan')
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  arguments=bridged, remappings=remaps,
                  parameters=[{'use_sim_time': True}])
    truth = Node(package='sim_gazebo', executable='contact_ground_truth.py', output='screen',
                 parameters=[{'use_sim_time': True, 'tcp_frame': fr['tcp'],
                              # gz reports contact points in the world; with base_motion there is no `world` link in TF,
                              # but `odom` starts at the spawn pose = the world origin (OdometryPublisher)
                              'world_frame': 'odom' if base_motion else 'world'}])

    actions = [gz, rsp, bridge, truth, spawn]
    # the same RViz window as the MuJoCo sim (mp_rviz view.launch.py); on by default, skipped without a display or when
    # mp_rviz is not installed, rviz:=false opts out
    if lc('rviz').lower() in ('1', 'true', 'yes') and (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        try:
            rviz_launch = os.path.join(get_package_share_directory('mp_rviz'), 'launch', 'view.launch.py')
        except Exception as e:
            print(f'sim_gazebo: RViz skipped, mp_rviz not found ({e})')
            rviz_launch = None
        if rviz_launch:
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rviz_launch),
                # this sim runs on /clock (robot_state_publisher, bridge and odom use sim time), so RViz must too, or
                # every TF lookup against wall time fails and nothing shows
                launch_arguments={'profile': lc('rviz_profile'), 'use_sim_time': 'true',
                                  'markers': 'true', 'scene': lc('rviz_scene'), 'fixed_frame': 'odom',
                                  'scene_frame': 'odom', 'plan_file': lc('rviz_plan_file'),
                                  'wall_scene_json': lc('rviz_wall_scene') or rviz_world_wall or lc('wall_scene'),
                                  'moveit_profile': lc('profile')}.items()))
    if (base_motion and lc('odom_realism') != 'off') or (depth and lc('depth_realism') != 'off'):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('sim_realism'), 'launch',
                                                       'realism.launch.py')),
            launch_arguments={'odom_realism': lc('odom_realism') if base_motion else 'off',
                              'depth_realism': lc('depth_realism') if depth else 'off',
                              'seed': lc('realism_seed'), 'overrides': lc('realism_overrides'),
                              'use_sim_time': 'true'}.items()))
    if lc('arm_meshes') == 'true':
        # gz resolves the description's model://ur_description/... URIs through GZ_SIM_RESOURCE_PATH; without it the UR links
        # have no visual and no collision geometry in gz (only the primitives of the lift, tool and sensor link exist)
        try:
            share = os.path.dirname(get_package_share_directory('ur_description'))
        except PackageNotFoundError:
            share = ''
            print('[sim_gazebo] arm_meshes:=true but ur_description is not installed: the UR links have no geometry in gz')
        if share:
            actions.insert(0, SetEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH', os.pathsep.join(p for p in (os.environ.get('GZ_SIM_RESOURCE_PATH', ''), share) if p)))
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
        DeclareLaunchArgument('rviz', default_value='true', description='open the same RViz window as the MuJoCo sim (mp_rviz); skipped without a display, rviz:=false opts out'),
        DeclareLaunchArgument('rviz_profile', default_value='sim_overview', description='mp_rviz profiles/<name>.rviz'),
        DeclareLaunchArgument('rviz_scene', default_value='true', description='also start mp_viz_scene.py (wall, plan columns, stands)'),
        DeclareLaunchArgument('rviz_plan_file', default_value='', description='coverage_output.json the manager executes (plan columns)'),
        DeclareLaunchArgument('rviz_wall_scene', default_value='', description='wall_scene.json for the wall display (default: wall_scene)'),
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
        DeclareLaunchArgument('position_gain', default_value='',
                              description='gz_ros2_control position_proportional_gain in 1/s (empty: the plugin default '
                                          '0.1, a 10 s time constant; see docs/sim_gazebo.md)'),
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
        DeclareLaunchArgument('lidar', default_value='false',
                              description='true: gz gpu_lidar on base_footprint, LaserScan on /scan in frame lidar_link '
                                          '(needs gz rendering, ogre2); false adds nothing'),
        DeclareLaunchArgument('lidar_pose', default_value='0.45 0 0.2 0 0 0',
                              description='"x y z roll pitch yaw" of lidar_link in base_footprint (x forward)'),
        DeclareLaunchArgument('lidar_angle_min_deg', default_value='-135.0', description='first ray, degrees'),
        DeclareLaunchArgument('lidar_angle_max_deg', default_value='135.0', description='last ray, degrees'),
        DeclareLaunchArgument('lidar_angle_increment_deg', default_value='0.5', description='ray spacing, degrees'),
        DeclareLaunchArgument('lidar_range_min_m', default_value='0.05', description='lidar minimum range, m'),
        DeclareLaunchArgument('lidar_range_max_m', default_value='10.0', description='lidar maximum range, m'),
        DeclareLaunchArgument('lidar_rate_hz', default_value='10.0', description='lidar rate, Hz'),
        DeclareLaunchArgument('lidar_noise_m', default_value='0.0', description='Gaussian range noise std of the lidar, m'),
        DeclareLaunchArgument('odom_realism', default_value='off',
                              description='off|good|typical|poor: wheel-odometry noise from sim_realism (needs base_motion:=true); '
                                          'gz odometry becomes /odom_truth, the node publishes /odom and TF (docs/sim_realism.md)'),
        DeclareLaunchArgument('depth_realism', default_value='off',
                              description='off|good|typical|poor: depth noise from sim_realism (needs depth_sensor:=true); '
                                          '/depth_sensor/points is the noisy cloud, the clean one is /depth_sensor/points_truth'),
        DeclareLaunchArgument('realism_seed', default_value='-1', description='sim_realism seed, -1 = random'),
        DeclareLaunchArgument('realism_overrides', default_value='',
                              description='"odom.<field>=<v> depth.<field>=<v>" on top of the sim_realism presets'),
        DeclareLaunchArgument('arm_meshes', default_value='true',
                              description='true (default): make the UR meshes resolvable in gz (visual AND collision); false: the '
                                          'UR links are invisible and have no collision in gz (the old behaviour)'),
        DeclareLaunchArgument('rgb_camera', default_value='false',
                              description='true: gz colour camera on base_footprint, Image on /rgb_camera/image in frame '
                                          'rgb_camera_link (needs gz rendering, ogre2); false adds nothing'),
        DeclareLaunchArgument('rgb_camera_pose', default_value='0.0 -2.4 1.6 0 0.11 1.38',
                              description='"x y z roll pitch yaw" of rgb_camera_link in base_footprint (x forward): '
                                          'the default looks at the wall in front of the arm from the right side, 2.4 m out'),
        DeclareLaunchArgument('rgb_camera_frame', default_value='base',
                              description='base: the camera is a link on base_footprint (moves with the base); world: a static '
                                          'model at rgb_camera_pose in the WORLD frame (stays put when the base drives, for '
                                          'filming a multi-stand pass)'),
        DeclareLaunchArgument('rgb_camera_size', default_value='1280 720', description='"width height", pixels'),
        DeclareLaunchArgument('rgb_camera_rate_hz', default_value='2.0', description='colour camera rate, Hz'),
        OpaqueFunction(function=_launch),
    ])
