"""Launches manipulation_manager_node with this robot's MoveIt parameters.

The node's MoveGroupInterface reads robot_description, the semantic description and kinematics
off its own node's parameters, so it needs the same parameters move_group gets; they come from
mobile_manipulator_moveit_config's shared builder, as in motion_planner_moveit's
coverage_pass_demo.launch.py. Frame and planning-group names come from the profile.

    ros2 launch manipulation_manager manipulation_manager.launch.py
    ros2 launch manipulation_manager manipulation_manager.launch.py profile:=panda_telescopic
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mobile_manipulator_moveit_config import build_moveit_config, resolve_profile


def _node(context):
    arm = LaunchConfiguration("arm").perform(context) or None
    profile = LaunchConfiguration("profile").perform(context) or None
    moveit_config = build_moveit_config(arm, hardware="mock", profile=profile)
    return [Node(
        package="manipulation_manager",
        executable="manipulation_manager_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"planning_pipelines.pipeline_names": ["ompl"]},
            {"profile": resolve_profile(arm, profile).name},
            # seed-preferring IK in the backend: 0 = one plain solve (see coverage_pass_demo.launch.py)
            {"ik_extra_seeds": int(LaunchConfiguration("ik_seeds").perform(context))},
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arm", default_value="",
                              description="arm name; picks the matching profile when `profile` is empty"),
        DeclareLaunchArgument("profile", default_value="",
                              description="robot profile (default ur10e_telescopic)"),
        DeclareLaunchArgument("ik_seeds", default_value="0", description="extra perturbed-seed IK solves (0 = off)"),
        OpaqueFunction(function=_node),
    ])
