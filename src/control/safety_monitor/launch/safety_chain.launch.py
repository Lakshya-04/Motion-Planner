"""Opt-in launch of the SafetyChain ROS node. NOT a certified safety function; nothing starts it by default.

  # thresholds from a tool profile (overload_abort_n, tangential_torque_abort_nm)
  ros2 launch safety_monitor safety_chain.launch.py tool_profiles_file:=/path/tool_profiles.yaml tool_name:=contact_wiper

  # no profile: both thresholds explicitly (0 disables a check)
  ros2 launch safety_monitor safety_chain.launch.py force_abort_n:=60.0 torque_abort_nm:=8.0

Reset after the cause is gone: ros2 service call /safety_chain_node/reset std_srvs/srv/Trigger
Parameters are documented in docs/safety_chain.md ('ROS node'). Nothing consumes the arm stop topic yet.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = {
    'tool_profiles_file': ('', str),
    'tool_name': ('', str),
    'force_abort_n': ('-1.0', float),
    'torque_abort_nm': ('-1.0', float),
    'ft_stale_s': ('0.1', float),
    'joint_stale_s': ('0.1', float),
    'require_ft': ('true', bool),
    'require_joint_state': ('true', bool),
    'hold_auto_clear': ('false', bool),
    'hold_clear_time_s': ('1.0', float),
    'rate_hz': ('50.0', float),
    'wrench_topic': ('/ft_conditioner/wrench_conditioned', str),
    'joint_states_topic': ('/joint_states', str),
    'estop_topic': ('/safety_chain/estop', str),
    'controller_fault_topic': ('/safety_chain/controller_fault', str),
    'cmd_vel_topic': ('/cmd_vel', str),
    'arm_stop_topic': ('/safety_chain/arm_stop', str),
    'state_topic': ('/safety_chain/state', str),
    'use_sim_time': ('false', bool),
}
# severity.<cause> = none | hold | estop (launch argument severity_<cause>)
CAUSES = ['force_overload', 'torque_overload', 'stale_ft', 'stale_joint_state', 'base_obstacle_hold',
          'base_obstacle_abort', 'operator_estop', 'controller_fault']
SEVERITY_DEFAULTS = {'force_overload': 'estop', 'torque_overload': 'estop', 'stale_ft': 'hold',
                     'stale_joint_state': 'estop', 'base_obstacle_hold': 'hold',
                     'base_obstacle_abort': 'estop', 'operator_estop': 'estop',
                     'controller_fault': 'estop'}


def _sev_arg(cause):
    return 'severity_' + cause


def _node(context):
    params = {k: ParameterValue(LaunchConfiguration(k), value_type=v[1]) for k, v in ARGS.items()}
    for c in CAUSES:
        params['severity.' + c] = ParameterValue(LaunchConfiguration(_sev_arg(c)), value_type=str)
    return [Node(package='safety_monitor', executable='safety_chain_node', output='screen',
                 parameters=[params])]


def generate_launch_description():
    declared = [DeclareLaunchArgument(k, default_value=v[0]) for k, v in ARGS.items()]
    declared += [DeclareLaunchArgument(_sev_arg(c), default_value=SEVERITY_DEFAULTS[c]) for c in CAUSES]
    return LaunchDescription(declared + [OpaqueFunction(function=_node)])
