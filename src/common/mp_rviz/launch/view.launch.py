"""Open RViz with one profile of this package: ``ros2 launch mp_rviz view.launch.py profile:=<name>``.

A profile is ``profiles/<name>.rviz``. ``profile:=list`` is not a launch feature: run
``ros2 launch mp_rviz view.launch.py --show-args`` for the arguments and see docs/rviz/README.md for the table.
``rviz_config:=/abs/path.rviz`` overrides the profile. ``markers:=true`` also starts mp_viz_markers.py
(mission state text and the tool trace coloured by force).
"""
import os

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
    sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('1', 'true', 'yes')
    actions = [Node(
        package='rviz2', executable='rviz2', name='rviz2_' + profile, output='screen',
        arguments=['-d', cfg], parameters=[{'use_sim_time': sim_time}])]
    if LaunchConfiguration('markers').perform(context).lower() in ('1', 'true', 'yes'):
        actions.append(Node(
            package='mp_rviz', executable='mp_viz_markers.py', name='mp_viz_markers', output='screen',
            parameters=[{'use_sim_time': sim_time}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='robot_model', description='profiles/<name>.rviz'),
        DeclareLaunchArgument('rviz_config', default_value='', description='absolute path; overrides profile'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('markers', default_value='false', description='also start mp_viz_markers.py'),
        OpaqueFunction(function=_setup),
    ])
