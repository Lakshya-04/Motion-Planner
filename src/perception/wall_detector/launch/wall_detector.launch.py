"""Wall detector: PointCloud2 -> wall plane -> wall_scene.json (+ walls.pcd) for the coverage planner.

    ros2 launch wall_detector wall_detector.launch.py cloud_topic:=/depth_sensor/points \
        target_frame:=base_footprint output_dir:=/tmp/wall_detector
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = {
        'cloud_topic': ('/depth_sensor/points', str, 'input sensor_msgs/PointCloud2'),
        'target_frame': ('base_footprint', str, 'frame the wall is expressed in (needs TF from the cloud frame)'),
        'output_dir': ('/tmp/wall_detector', str, 'where wall_scene.json and walls.pcd are written'),
        'wall_id': ('0', int, 'id of the wall in the written scene'),
        'accumulate_frames': ('5', int, 'clouds merged per fit'),
        'inlier_threshold': ('0.01', float, 'RANSAC point-to-plane threshold, m'),
        'max_range_m': ('0.0', float, 'ignore points farther than this from the sensor (0 = no limit)'),
        'publish_collision_object': ('false', bool, 'also add a box at the wall to the MoveIt planning scene'),
    }
    declared = [DeclareLaunchArgument(k, default_value=v[0], description=v[2]) for k, v in args.items()]
    params = {k: ParameterValue(LaunchConfiguration(k), value_type=v[1]) for k, v in args.items()}
    node = Node(package='wall_detector', executable='wall_detector_node', name='wall_detector',
                output='screen', parameters=[params])
    # Octomap -> distance field for the planners (needs move_group with live_octomap:=true).
    esdf = Node(package='wall_detector', executable='octomap_esdf_node', name='octomap_esdf', output='screen',
                condition=IfCondition(LaunchConfiguration('esdf')),
                parameters=[{'center': [0.8, 0.0, 1.0], 'dims': [1.6, 1.6, 1.6],
                             'voxel_size': ParameterValue(LaunchConfiguration('esdf_voxel_size'), value_type=float)}])
    return LaunchDescription([
        DeclareLaunchArgument('esdf', default_value='false',
                              description='also run octomap_esdf_node (planning-scene octomap -> ~/esdf)'),
        DeclareLaunchArgument('esdf_voxel_size', default_value='0.05', description='ESDF voxel size, m'),
        *declared, node, esdf])
