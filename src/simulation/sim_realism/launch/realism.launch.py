"""Odometry and depth realism nodes, standalone or included by the simulator launches.

    ros2 launch sim_realism realism.launch.py odom_realism:=typical depth_realism:=good seed:=7
    ros2 launch sim_realism realism.launch.py overrides:="odom.scale_lin_sigma=0.02 depth.sigma_a=0.002"

odom_realism  off|good|typical|poor: /odom_truth -> /odom (+ TF odom -> base_footprint); off starts no odom node
depth_realism off|good|typical|poor: /depth_sensor/points_truth -> /depth_sensor/points; off starts no depth node
overrides     "odom.<field>=<value> depth.<field>=<value> ..." on top of the presets (fields: sim_realism/presets.py)
Run with the simulator launched with the same two arguments, so that it publishes the truth topics.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _overrides(text, prefix):
    out = {}
    for item in text.replace(',', ' ').split():
        key, _, value = item.partition('=')
        if key.startswith(prefix + '.'):
            out[key[len(prefix) + 1:]] = int(value) if key.endswith('latency_periods') else float(value)
    return out


def _nodes(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    common = {'seed': int(arg('seed')), 'use_sim_time': arg('use_sim_time') == 'true'}
    nodes = []
    if arg('odom_realism') != 'off':
        nodes.append(Node(package='sim_realism', executable='odom_realism_node', name='odom_realism', output='screen',
                          parameters=[{**common, 'preset': arg('odom_realism'), 'publish_tf': arg('publish_tf') == 'true',
                                       **_overrides(arg('overrides'), 'odom')}]))
    if arg('depth_realism') != 'off':
        nodes.append(Node(package='sim_realism', executable='depth_realism_node', name='depth_realism', output='screen',
                          parameters=[{**common, 'preset': arg('depth_realism'), **_overrides(arg('overrides'), 'depth')}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('odom_realism', default_value='typical', description='off|good|typical|poor'),
        DeclareLaunchArgument('depth_realism', default_value='typical', description='off|good|typical|poor'),
        DeclareLaunchArgument('seed', default_value='-1', description='random seed, -1 = a new one each run (logged)'),
        DeclareLaunchArgument('publish_tf', default_value='true', description='odom node publishes TF odom -> base_footprint'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='true for Gazebo'),
        DeclareLaunchArgument('overrides', default_value='', description='"odom.<field>=<v> depth.<field>=<v> ..."'),
        OpaqueFunction(function=_nodes),
    ])
