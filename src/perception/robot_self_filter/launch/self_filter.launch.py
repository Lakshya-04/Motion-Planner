"""Run the self filter on one cloud topic.

  ros2 launch robot_self_filter self_filter.launch.py cloud_in:=/lidar/points cloud_out:=/lidar/points_filtered
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('robot_self_filter'), 'config', 'self_filter.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('cloud_in', default_value='cloud_in'),
        DeclareLaunchArgument('cloud_out', default_value='cloud_out'),
        Node(package='robot_self_filter', executable='self_filter_node', name='self_filter',
             output='screen', parameters=[config],
             remappings=[('cloud_in', LaunchConfiguration('cloud_in')),
                         ('cloud_out', LaunchConfiguration('cloud_out'))]),
    ])
