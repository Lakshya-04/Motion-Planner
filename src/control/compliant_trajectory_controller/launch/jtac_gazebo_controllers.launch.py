"""Spawn joint_state_broadcaster + JtacController (combined_arm_lift_controller) against a
controller_manager that's already running -- specifically, gz_ros2_control's embedded one inside
gzserver, brought up by simulator's gazebo.launch.py (arm_control_mode:=joint_trajectory) via
robot.urdf.xacro's <gazebo> plugin tag. Deliberately does NOT start its own ros2_control_node or
robot_state_publisher -- those already exist by the time this runs; see gazebo.launch.py's own
module docstring ("NO controllers are spawned... a separate launch file's job") for why the split
is exactly here.

combined_arm_lift_controller's own params (config/gz_controllers.yaml) are already loaded into
the live controller_manager -- gz_ros2_control's plugin loaded that whole file at model-spawn
time. robot_description is re-overridden on THIS controller's own node via --param-file:
JtacController reads it via admittance_controller::ParamListener on its own node, not on
/controller_manager (same reason as jtac_controller_test.launch.py's own --param-file, and the
same duplicated render -- see that file's header for why it isn't imported instead).

ft_sensor.name is left at gz_controllers.yaml's real value (uf_ft_sensor) here: add_ft_sensor_bridge
is passed to the xacro render below, which - under arm_control_mode:=joint_trajectory - enables the
native gz-sim force_torque joint sensor at joint_eef (see uf850_joint_trajectory_ros2_control.xacro's
own header), not the ft_bridge_hardware topic bridge that can't load under gz_ros2_control's
resource manager. `ros2 control list_hardware_interfaces` should now show real uf_ft_sensor/*
state interfaces, so JtacController's ForceTorqueSensor wrapper finds them and has_ft_sensor_ is
true (see jtac_controller.cpp's own on_configure()) -- real Gazebo-physics force feedback, not the
degraded no-F/T mode this test used before.

Usage:
    ros2 launch simulator gazebo.launch.py base_type:=serum arm_control_mode:=joint_trajectory
    ros2 launch compliant_trajectory_controller jtac_gazebo_controllers.launch.py
"""
import os
import subprocess
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def _render_robot_description() -> str:
    """Renders the SAME robot_description simulator/gazebo.launch.py -> description's
    description.launch.py builds for this test (base_type:=serum, use_sim:=true,
    arm_control_mode:=joint_trajectory, add_ft_sensor_bridge:=true) -- see this module's own
    docstring for why this is re-rendered here rather than fetched from the live
    /robot_state_publisher, and for why add_ft_sensor_bridge is true here specifically.
    """
    import xacro
    from ament_index_python.packages import get_package_share_directory

    xacro_path = os.path.join(
        get_package_share_directory("description"), "urdf", "robot.urdf.xacro")
    return xacro.process_file(
        xacro_path,
        mappings={
            "base_type": "serum",
            "use_sim": "true",
            "arm_control_mode": "joint_trajectory",
            "add_ft_sensor_bridge": "true",
        },
    ).toxml()


def generate_launch_description():
    robot_description_content = _render_robot_description()

    controller_param_file_fd, controller_param_file_path = tempfile.mkstemp(
        prefix="jtac_gazebo_controller_params_", suffix=".yaml")
    with os.fdopen(controller_param_file_fd, "w") as f:
        yaml.safe_dump(
            {"combined_arm_lift_controller": {
                "ros__parameters": {
                    "robot_description": robot_description_content,
                }}},
            f)

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    jtac_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "combined_arm_lift_controller", "--controller-manager", "/controller_manager",
            "--param-file", controller_param_file_path],
        output="screen",
    )

    delay_jtac_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner, on_exit=[jtac_spawner]))

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        delay_jtac_spawner,
    ])
