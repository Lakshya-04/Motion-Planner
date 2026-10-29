from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    wall_scene_json_arg = DeclareLaunchArgument(
        "wall_scene_json",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("coverage_planner"),
                "test",
                "data",
                "wall_scene.json",
            ]
        ),
        description=(
            "Path to a wall_scene.json export (its own directory must also hold that wall's "
            "walls.pcd). Read as a node parameter, not a service request: ~/start is a "
            "std_srvs/Trigger, which carries no fields."
        ),
    )
    wall_id_arg = DeclareLaunchArgument(
        "wall_id",
        default_value="0",
        description="Which wall (by id) in wall_scene_json to plan.",
    )
    resolution_arg = DeclareLaunchArgument(
        "resolution",
        default_value="0.05",
        description="Rasterization grid resolution (m/cell).",
    )
    add_strip_connectors_arg = DeclareLaunchArgument(
        "add_strip_connectors",
        default_value="false",
        choices=["true", "false"],
        description=(
            "Interpolate a real connector waypoint chain from one strip's end to the next "
            "strip's start."
        ),
    )
    allow_unknown_arg = DeclareLaunchArgument(
        "allow_unknown",
        default_value="false",
        choices=["true", "false"],
        description="Treat cells the wall's own scan never covered as free.",
    )

    tool_name_arg = DeclareLaunchArgument("tool_name", default_value="mop")
    contact_arg = DeclareLaunchArgument(
        "contact",
        default_value="true",
        choices=["true", "false"],
        description="True for a tool physically touching the wall.",
    )
    swath_m_arg = DeclareLaunchArgument(
        "swath_m",
        default_value="0.2",
        description="Effective tool footprint width (m).",
    )
    sweep_direction_arg = DeclareLaunchArgument(
        "sweep_direction",
        default_value="vertical",
        choices=["vertical", "horizontal"],
    )
    enable_boundary_pass_arg = DeclareLaunchArgument(
        "enable_boundary_pass", default_value="false", choices=["true", "false"]
    )
    double_pass_arg = DeclareLaunchArgument(
        "double_pass",
        default_value="false",
        choices=["true", "false"],
        description="Two identical-standoff passes, second one reversed (e.g. a mop).",
    )

    coverage_planner_node = Node(
        package="coverage_planner",
        executable="coverage_planner_node",
        name="coverage_planner_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("coverage_planner"),
                    "config",
                    "coverage_planner.yaml",
                ]
            ),
            {"plan_all_components": True},
        ],
    )

    wall_coverage_service_node = Node(
        package="coverage_planner",
        executable="wall_coverage_service_node",
        name="wall_coverage_service_node",
        output="screen",
        parameters=[
            {
                "wall_scene_json": LaunchConfiguration("wall_scene_json"),
                "wall_id": LaunchConfiguration("wall_id"),
                "resolution": LaunchConfiguration("resolution"),
                "add_strip_connectors": LaunchConfiguration("add_strip_connectors"),
                "allow_unknown": LaunchConfiguration("allow_unknown"),
                "tool_name": LaunchConfiguration("tool_name"),
                "contact": LaunchConfiguration("contact"),
                "swath_m": LaunchConfiguration("swath_m"),
                "sweep_direction": LaunchConfiguration("sweep_direction"),
                "enable_boundary_pass": LaunchConfiguration("enable_boundary_pass"),
                "double_pass": LaunchConfiguration("double_pass"),
            }
        ],
    )

    return LaunchDescription(
        [
            wall_scene_json_arg,
            wall_id_arg,
            resolution_arg,
            add_strip_connectors_arg,
            allow_unknown_arg,
            tool_name_arg,
            contact_arg,
            swath_m_arg,
            sweep_direction_arg,
            enable_boundary_pass_arg,
            double_pass_arg,
            coverage_planner_node,
            wall_coverage_service_node,
        ]
    )
