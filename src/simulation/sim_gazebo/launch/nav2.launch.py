"""Nav2 servers for the omni base, odom frame, no map (docs/nav2.md).

Start sim.launch.py with base_motion:=true lidar:=true (depth_sensor:=true for the depth source) first.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SERVERS = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']
PACKAGES = {'controller_server': 'nav2_controller', 'planner_server': 'nav2_planner',
            'behavior_server': 'nav2_behaviors', 'bt_navigator': 'nav2_bt_navigator'}


def generate_launch_description():
    default = os.path.join(get_package_share_directory('sim_gazebo'), 'config', 'nav2', 'nav2_omni_params.yaml')
    nodes = [Node(package=PACKAGES[n], executable=n, name=n, output='screen',
                  parameters=[LaunchConfiguration('params_file')]) for n in SERVERS]
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
                      output='screen', parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': SERVERS,
                                                    'bond_timeout': 10.0}]))
    return LaunchDescription([DeclareLaunchArgument('params_file', default_value=default)] + nodes)
