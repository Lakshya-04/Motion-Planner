"""Launch JtacController under a REAL controller_manager for this repo's actual 7-DOF arm+lift
chain (description, base_type:=serum), with a mock F/T sensor (ft_bridge_hardware,
injection_mode) wired in so a wall-contact scenario can be scripted without touching hardware.

Ported from an earlier internal repo, where this rendered robot_description's own
robot.urdf.xacro. That package doesn't exist in this repo -- see
moveit_config/README.md and this branch's own port history for why it wasn't brought over
wholesale (this repo's description already replaces it, more maturely). The one real gap that
surfaced: sii's arm ros2_control was still the vendor's old joint-position-command interface;
this repo's current arm/uf850_ros2_control.xacro is joint-state-only by design (the arm's onboard
force-control firmware runs Cartesian tcp+ft GPIO commands instead -- see that file's own header).
description/urdf/robot.urdf.xacro's new arm_control_mode:=joint_trajectory arg (added
alongside this port) is the fix: a second, explicitly non-production ros2_control interface
(description/arm/uf850_joint_trajectory_ros2_control.xacro) with plain joint position/
velocity command+state interfaces, for exactly this kind of joint-trajectory-controller testing --
see that file's own header for why it's kept separate from the production interface
arm_lift_controller/real hardware use.

Deliberately self-contained rather than modifying any shared production launch/config: robot_
description is not automatically visible to an individual controller's own node (it lands on
/controller_manager, not on combined_arm_lift_controller's own node) -- handling that correctly
here, in isolation, avoids risking a regression in any shared path while this controller is still
a first, disclosed-scope-cut increment (see jtac_controller.hpp's own doc: no per-segment
engage/disengage yet, no path/goal tolerance checking).

This test drives the controller directly via its FollowJointTrajectory action (scripts/
send_test_trajectory_and_inject_wrench.py) rather than through MoveGroupInterface -- proving the
controller itself is a genuine, correctly-deployed controller_manager plugin first. Wiring the
full MoveIt path through it is the next step, not attempted here.

Usage:
    ros2 launch compliant_trajectory_controller jtac_controller_test.launch.py
    ros2 run compliant_trajectory_controller send_test_trajectory_and_inject_wrench.py
"""
import os
import subprocess
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# FT sensor's own <sensor name="..."> value -- must match config/jtac_controller.yaml's own
# ft_sensor.name so JtacController's semantic_components::ForceTorqueSensor finds the right
# state interfaces, and description/urdf/robot.urdf.xacro's own ft_sensor_name arg default.
_FT_SENSOR_NAME = "uf_ft_sensor"


def _render_real_robot_description() -> str:
    """Renders description/urdf/robot.urdf.xacro (base_type:=serum, the default -- lift +
    UF850 arm + mop tool) with arm_control_mode:=joint_trajectory (that xacro's own non-production
    joint-command interface -- see its header) and add_ft_sensor_bridge:=true/
    ft_sensor_injection_mode:=true (ft_bridge_hardware wired in as a mock F/T sensor).
    """
    pkg_desc_dir = get_package_share_directory("description")
    xacro_path = os.path.join(pkg_desc_dir, "urdf", "robot.urdf.xacro")
    command = [
        "xacro",
        xacro_path,
        "base_type:=serum",
        "use_mock_hardware:=true",
        "arm_control_mode:=joint_trajectory",
        "add_ft_sensor_bridge:=true",
        f"ft_sensor_name:={_FT_SENSOR_NAME}",
        "ft_sensor_injection_mode:=true",
    ]
    return subprocess.run(command, check=True, capture_output=True, text=True).stdout


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("compliant_trajectory_controller")
    robot_description_content = _render_real_robot_description()
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # robot_description on THIS controller's own node: admittance_controller::ParamListener (and
    # therefore JtacController) reads it as a plain parameter on get_node(), which is
    # "combined_arm_lift_controller" -- passing it only to ros2_control_node's own /controller_
    # manager parameters (as the base robot_description dict above already is) would NOT make it
    # visible here. See config/jtac_controller.yaml's own comment and force_control_backend_
    # admittance_vs_diffik.md's Phase 2 step 4 for why this needs a separate --param-file.
    # T11 -- see jtac_controller.hpp's own header doc. Same comma-separated-string convention
    # jtac_compliance_demo.launch.py's own redundancy_weights arg uses (ros2 launch has no
    # native list-typed CLI argument).
    redundancy_weights = [
        float(w) for w in LaunchConfiguration("redundancy_weights").perform(context).split(",")
    ]

    controller_param_file_fd, controller_param_file_path = tempfile.mkstemp(
        prefix="jtac_controller_params_", suffix=".yaml")
    with os.fdopen(controller_param_file_fd, "w") as f:
        yaml.safe_dump(
            {"combined_arm_lift_controller": {
                "ros__parameters": {
                    "robot_description": robot_description_content,
                    "enable_redundancy_weighting": LaunchConfiguration(
                        "enable_redundancy_weighting").perform(context) == "true",
                    "redundancy_weights": redundancy_weights,
                }}},
            f)

    controller_config_file = os.path.join(pkg_share, "config", "jtac_controller.yaml")

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config_file],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

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

    rviz_config_path = os.path.join(pkg_share, "rviz", "jtac_compliance_demo.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(LaunchConfiguration("show_rviz")),
        output="screen",
    )

    return [
        ros2_control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        delay_jtac_spawner,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "show_rviz", default_value="true",
                description="Launch RViz alongside the test (per this repo's convention: verify "
                "live, not headless)."),
            DeclareLaunchArgument(
                "enable_redundancy_weighting", default_value="false",
                description="T11 -- see jtac_controller.hpp's own header doc. Default false: "
                "byte-identical to before this landed."),
            DeclareLaunchArgument(
                "redundancy_weights", default_value="10.0,1.0,1.0,1.0,1.0,1.0,1.0",
                description="enable_redundancy_weighting:=true only. Comma-separated, 7 values "
                "(lift, joint1..6) -- see jtac_controller.hpp's own header doc for the design "
                "choice this default represents."),
            OpaqueFunction(function=_launch_setup),
        ]
    )
