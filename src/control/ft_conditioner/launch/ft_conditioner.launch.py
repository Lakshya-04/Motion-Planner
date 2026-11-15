"""Condition a raw force/torque sensor wrench into the contact wrench at the tool.

  # Gazebo backend (raw wrench on /force_torque_sensor_broadcaster/wrench, frame ft_sensor_link)
  ros2 launch ft_conditioner ft_conditioner.launch.py input_topic:=/force_torque_sensor_broadcaster/wrench \\
      payload_mass_kg:=0.8

  # MuJoCo backend (raw wrench on /sim_mujoco/ft_raw)
  ros2 launch ft_conditioner ft_conditioner.launch.py input_topic:=/sim_mujoco/ft_raw payload_mass_kg:=0.8

A simulator with no TF frame for its sensor: tool_in_sensor_from_tf:=false tool_in_sensor_xyz:='[0,0,0.1]'
tool_in_sensor_rpy:='[0,0,0]'. A calibrated payload: payload_mass_kg, payload_cog_xyz:='[x,y,z]', bias:='[6 values]'.

The output (frame = tool_frame) goes to `output_topic`; point ft_bridge_hardware's source_topic or
injection_topic at it. Calibrate with the ~/capture_pose and ~/identify services (docs/ft_sensing.md).
"""
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
    'tool_in_sensor_from_tf': ('true', bool),
    'max_tf_age': ('0.1', float),
    'use_sim_time': ('false', bool),
}
# number lists, given as '[a, b, c]' (ints are fine: they become doubles)
LIST_ARGS = {
    'payload_cog_xyz': '[0.0, 0.0, 0.0]',
    'bias': '[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
    'tool_in_sensor_xyz': '[0.0, 0.0, 0.0]',
    'tool_in_sensor_rpy': '[0.0, 0.0, 0.0]',
}


def _node(context):
    params = {k: ParameterValue(LaunchConfiguration(k), value_type=v[1]) for k, v in ARGS.items()}
    for k in LIST_ARGS:
        params[k] = [float(x) for x in yaml.safe_load(LaunchConfiguration(k).perform(context))]
    return [Node(package='ft_conditioner', executable='wrench_conditioner_node', output='screen',
                 parameters=[params])]


def generate_launch_description():
    declared = [DeclareLaunchArgument(k, default_value=v[0]) for k, v in ARGS.items()]
    declared += [DeclareLaunchArgument(k, default_value=v) for k, v in LIST_ARGS.items()]
    return LaunchDescription(declared + [OpaqueFunction(function=_node)])
