"""Condition a raw force/torque sensor wrench into the contact wrench at the tool.

  # Gazebo backend (raw wrench on /force_torque_sensor_broadcaster/wrench, frame ft_sensor_link)
  ros2 launch ft_conditioner ft_conditioner.launch.py input_topic:=/force_torque_sensor_broadcaster/wrench \\
      payload_mass_kg:=0.8

  # MuJoCo backend (raw wrench on /sim_mujoco/ft_raw)
  ros2 launch ft_conditioner ft_conditioner.launch.py input_topic:=/sim_mujoco/ft_raw payload_mass_kg:=0.8

The output (frame = tool_frame) goes to `output_topic`; point ft_bridge_hardware's source_topic or
injection_topic at it. Calibrate with the ~/capture_pose and ~/identify services (docs/ft_sensing.md).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = {
    'input_topic': ('wrench_raw', str),
    'output_topic': ('wrench_conditioned', str),
    'world_frame': ('base_footprint', str),
    'tool_frame': ('tcp_link', str),
    'sensor_frame': ('', str),
    'sign': ('1.0', float),
    'payload_mass_kg': ('0.0', float),
    'use_sim_time': ('false', bool),
}


def generate_launch_description():
    declared = [DeclareLaunchArgument(k, default_value=v[0]) for k, v in ARGS.items()]
    params = {k: ParameterValue(LaunchConfiguration(k), value_type=v[1]) for k, v in ARGS.items()}
    return LaunchDescription(declared + [
        Node(package='ft_conditioner', executable='wrench_conditioner_node', output='screen',
             parameters=[params]),
    ])
