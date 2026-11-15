"""Launches manipulation_manager_node with the same MoveIt config params move_group.launch.py
supplies -- see motion_planner/launch/coverage_pass_demo.launch.py's own header for why.

    ros2 launch manipulation_manager manipulation_manager.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_share = get_package_share_directory("description")
    urdf_xacro_path = os.path.join(description_share, "urdf", "robot.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder("robot", package_name="moveit_config")
        .robot_description(
            file_path=urdf_xacro_path,
            mappings={"base_type": "ranger_mini", "manipulation": "true"},
        )
        .robot_description_semantic(file_path="srdf/robot.srdf.xacro")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    manipulation_manager_node = Node(
        package="manipulation_manager",
        executable="manipulation_manager_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"planning_pipelines.pipeline_names": ["ompl"]},
            # seed-preferring IK in the backend: 0 = one plain solve (see coverage_pass_demo.launch.py)
            {"ik_extra_seeds": ParameterValue(LaunchConfiguration("ik_seeds"), value_type=int)},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("ik_seeds", default_value="0", description="extra perturbed-seed IK solves (0 = off)"),
        manipulation_manager_node,
    ])
