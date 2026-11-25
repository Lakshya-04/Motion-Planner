"""Open RViz with one profile of this package: ``ros2 launch mp_rviz view.launch.py profile:=<name>``.

A profile is ``profiles/<name>.rviz``. ``profile:=list`` is not a launch feature: run
``ros2 launch mp_rviz view.launch.py --show-args`` for the arguments and see docs/rviz/README.md for the table.
``rviz_config:=/abs/path.rviz`` overrides the profile. ``markers:=true`` also starts mp_viz_markers.py
(mission state text and the tool trace coloured by force). ``scene:=true`` starts mp_viz_scene.py (planned wall
columns, base stands). For the wall and the executed plan pass ``plan_file:=<coverage_output.json>`` and
``wall_scene_json:=<wall_scene.json>``; ``scene_frame`` (default ``odom``) is the frame those files' poses are meant in
(the scene file's own ``base_link`` is only right while the base is at the odom origin), ``fixed_frame`` (default
``odom``) is the frame the tool trace is drawn in. ``moveit_profile:=<description profile>`` gives the RViz node the
MoveIt parameters (robot description, SRDF, kinematics) that the Trajectory and PlanningScene displays need.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    share = get_package_share_directory('mp_rviz')
    cfg = LaunchConfiguration('rviz_config').perform(context)
    profile = LaunchConfiguration('profile').perform(context)
    if not cfg:
        cfg = os.path.join(share, 'profiles', profile + '.rviz')
    if not os.path.isfile(cfg):
        avail = sorted(f[:-5] for f in os.listdir(os.path.join(share, 'profiles')) if f.endswith('.rviz'))
        raise RuntimeError('no RViz profile "%s"; available: %s' % (profile, ', '.join(avail)))
    if LaunchConfiguration('moveit_displays').perform(context).lower() in ('1', 'true', 'yes'):
        import yaml
        with open(cfg) as f:
            doc = yaml.safe_load(f)
        for d in doc['Visualization Manager']['Displays']:
            if str(d.get('Class', '')).startswith('moveit_rviz_plugin/'):
                d['Enabled'] = True
                d['Value'] = True
        cfg = os.path.join(tempfile.mkdtemp(prefix='mp_rviz_'), profile + '_moveit.rviz')
        with open(cfg, 'w') as f:
            yaml.safe_dump(doc, f, sort_keys=False)
    sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('1', 'true', 'yes')
    rviz_params = [{'use_sim_time': sim_time}]
    moveit_profile = LaunchConfiguration('moveit_profile').perform(context)
    if moveit_profile:
        try:
            from mobile_manipulator_moveit_config import build_moveit_config
            mc = build_moveit_config(profile=moveit_profile, hardware='jtac_mock')
            rviz_params += [mc.robot_description, mc.robot_description_semantic, mc.robot_description_kinematics,
                            mc.planning_pipelines, mc.joint_limits]
        except Exception as e:  # the Trajectory / PlanningScene displays stay empty, the rest of the window works
            print('mp_rviz: no MoveIt parameters for RViz (%s)' % e)
    actions = [Node(
        package='rviz2', executable='rviz2', name='rviz2_' + profile, output='screen',
        arguments=['-d', cfg], parameters=rviz_params)]
    if LaunchConfiguration('markers').perform(context).lower() in ('1', 'true', 'yes'):
        actions.append(Node(
            package='mp_rviz', executable='mp_viz_markers.py', name='mp_viz_markers', output='screen',
            parameters=[{'use_sim_time': sim_time,
                         'fixed_frame': LaunchConfiguration('fixed_frame').perform(context)}]))
    if LaunchConfiguration('scene').perform(context).lower() in ('1', 'true', 'yes'):
        actions.append(Node(
            package='mp_rviz', executable='mp_viz_scene.py', name='mp_viz_scene', output='screen',
            parameters=[{'use_sim_time': sim_time, 'frame_id': LaunchConfiguration('scene_frame').perform(context),
                         'plan_file': LaunchConfiguration('plan_file').perform(context),
                         'wall_scene_json': LaunchConfiguration('wall_scene_json').perform(context),
                         'wall_mjcf': LaunchConfiguration('wall_mjcf').perform(context)}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='robot_model', description='profiles/<name>.rviz'),
        DeclareLaunchArgument('rviz_config', default_value='', description='absolute path; overrides profile'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('markers', default_value='false', description='also start mp_viz_markers.py'),
        DeclareLaunchArgument('scene', default_value='false', description='also start mp_viz_scene.py'),
        DeclareLaunchArgument('fixed_frame', default_value='odom', description='frame of the tool trace (markers:=true)'),
        DeclareLaunchArgument('scene_frame', default_value='odom', description='frame the plan_file / wall poses are in'),
        DeclareLaunchArgument('plan_file', default_value='', description='coverage_output.json for mp_viz_scene'),
        DeclareLaunchArgument('wall_scene_json', default_value='', description='wall_scene.json for mp_viz_scene'),
        DeclareLaunchArgument('wall_mjcf', default_value='',
                              description='MuJoCo MJCF whose geom wall_0 is drawn as the wall (the wall the sim really has)'),
        DeclareLaunchArgument('moveit_profile', default_value='',
                              description='description profile: gives RViz the MoveIt parameters (Trajectory display)'),
        DeclareLaunchArgument('moveit_displays', default_value='false',
                              description='enable the moveit_rviz_plugin displays (can block RViz startup)'),
        OpaqueFunction(function=_setup),
    ])
