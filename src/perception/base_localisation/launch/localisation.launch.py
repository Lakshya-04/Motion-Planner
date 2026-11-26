"""EKF localisation of the base: wheel odometry + IMU (+ lidar scan matching) -> /odometry/filtered and TF odom -> base.

    ros2 launch base_localisation localisation.launch.py platform:=omni
    ros2 launch base_localisation localisation.launch.py platform:=diff imu:=false odom_topic:=/wheel/odom
    ros2 launch base_localisation localisation.launch.py platform:=omni scan_match:=true use_sim_time:=true

platform     diff | omni | ackermann: which config/ekf_<platform>.yaml
imu          true: fuse the gyro yaw rate from imu_topic; false: wheel odometry only
scan_match   true: also start scan_match_node (/scan -> /scan_odom velocity) and fuse it
publish_tf   false when something else already publishes odom -> base (the simulators' realism node does by default)
name         node name; run more than one filter with publish_tf:=false on the extra ones and their own output_topic
params_file  optional extra YAML on top of the generated parameters
The filter output goes to output_topic; point consumers there (stand_navigator: odom_topic:=/odometry/filtered).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from base_localisation import ekf_params


def _nodes(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    on = lambda n: arg(n).lower() in ('true', '1')  # noqa: E731
    sim_time = on('use_sim_time')
    params = ekf_params.build(arg('platform'), imu=on('imu'), scan_match=on('scan_match'), odom_topic=arg('odom_topic'),
                              imu_topic=arg('imu_topic'), scan_odom_topic=arg('scan_odom_topic'),
                              publish_tf=on('publish_tf'), use_sim_time=sim_time)
    sources = [params] + ([arg('params_file')] if arg('params_file') else [])
    nodes = [Node(package='robot_localization', executable='ekf_node', name=arg('name'), output='screen',
                  parameters=sources, remappings=[('odometry/filtered', arg('output_topic'))])]
    if on('scan_match'):
        nodes.append(Node(package='base_localisation', executable='scan_match_node', name='scan_match', output='screen',
                          parameters=[{'scan_topic': arg('scan_topic'), 'output_topic': arg('scan_odom_topic'),
                                       'use_sim_time': sim_time}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value=ekf_params.NODE, description='EKF node name (several can run side by side)'),
        DeclareLaunchArgument('platform', default_value='omni', description='diff | omni | ackermann'),
        DeclareLaunchArgument('imu', default_value='true', description='fuse the IMU yaw rate'),
        DeclareLaunchArgument('scan_match', default_value='false', description='fuse lidar scan matching'),
        DeclareLaunchArgument('odom_topic', default_value='/odom', description='wheel odometry in'),
        DeclareLaunchArgument('imu_topic', default_value='/imu', description='sensor_msgs/Imu in'),
        DeclareLaunchArgument('scan_topic', default_value='/scan', description='LaserScan for scan matching'),
        DeclareLaunchArgument('scan_odom_topic', default_value='/scan_odom', description='scan matcher velocity'),
        DeclareLaunchArgument('output_topic', default_value='/odometry/filtered', description='filtered odometry out'),
        DeclareLaunchArgument('publish_tf', default_value='true', description='publish TF odom -> base_footprint'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='true for Gazebo'),
        DeclareLaunchArgument('params_file', default_value='', description='extra EKF parameter YAML (optional)'),
        OpaqueFunction(function=_nodes),
    ])
