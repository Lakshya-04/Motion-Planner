import json

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # RViz's Fixed Frame is a static field in the saved .rviz config, but different
    # wall_scene_json exports carry different frame_ids (e.g. a LiDAR's own "livox_frame_1").
    # Read the actual frame_id here and pass it to rviz2's own -f flag so it always matches.
    wall_scene_json_path = LaunchConfiguration("wall_scene_json").perform(context)
    fixed_frame = "link_base"
    try:
        with open(wall_scene_json_path, "r") as f:
            fixed_frame = json.load(f).get("frame_id", fixed_frame)
    except (OSError, json.JSONDecodeError) as e:
        print(
            f"wall_coverage_demo.launch.py: could not read frame_id from "
            f"'{wall_scene_json_path}' ({e}) -- defaulting RViz Fixed Frame to '{fixed_frame}'."
        )

    coverage_planner_node = Node(
        package="coverage_planner",
        executable="coverage_planner_node",
        name="coverage_planner_node",
        output="screen",
        # Without a params file, the node falls back to plan_all_components=false (its own code
        # default) -- on this demo's fixture the scan gaps split the wall into several disconnected
        # observed patches, so silently only the largest one would get covered. Override it so the
        # demo covers the whole wall, matching what an operator watching RViz actually expects.
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

    wall_coverage_demo_node = Node(
        package="coverage_planner",
        executable="wall_coverage_demo_node",
        name="wall_coverage_demo",
        output="screen",
        parameters=[
            {
                "wall_scene_json": LaunchConfiguration("wall_scene_json"),
                "wall_id": LaunchConfiguration("wall_id"),
                "resolution": LaunchConfiguration("resolution"),
                "coverage_service_name": LaunchConfiguration("coverage_service_name"),
                "allow_flatness_override": LaunchConfiguration(
                    "allow_flatness_override"
                ),
                "output_json": LaunchConfiguration("output_json"),
                "add_strip_connectors": LaunchConfiguration("add_strip_connectors"),
                "allow_unknown": LaunchConfiguration("allow_unknown"),
                "orientation_format": LaunchConfiguration("orientation_format"),
                "tool_name": LaunchConfiguration("tool_name"),
                "contact": LaunchConfiguration("contact"),
                "standoff_m": LaunchConfiguration("standoff_m"),
                "tool_normal_policy": LaunchConfiguration("tool_normal_policy"),
                "max_edge_tilt_deg": LaunchConfiguration("max_edge_tilt_deg"),
                "tool_frame_yaw_deg": LaunchConfiguration("tool_frame_yaw_deg"),
                "swath_m": LaunchConfiguration("swath_m"),
                "overlap_pct": LaunchConfiguration("overlap_pct"),
                "sweep_direction": LaunchConfiguration("sweep_direction"),
                "enable_boundary_pass": LaunchConfiguration("enable_boundary_pass"),
                "boundary_offset_m": LaunchConfiguration("boundary_offset_m"),
                "obstacle_inflation": LaunchConfiguration("obstacle_inflation"),
                "double_pass": LaunchConfiguration("double_pass"),
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("coverage_planner"),
                    "config",
                    "wall_coverage_demo.rviz",
                ]
            ),
            "-f",
            fixed_frame,
        ],
        condition=IfCondition(LaunchConfiguration("show_rviz")),
    )

    return [coverage_planner_node, wall_coverage_demo_node, rviz_node]


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
            "walls.pcd, same basename convention wall_export/ always used). Defaults to this "
            "package's own real-perception-export test fixture."
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
        description=(
            "Rasterization grid resolution (m/cell). Coverage_planner_node's own boustrophedon "
            "obstacle-splitting scales badly below ~0.05m on real wall/obstacle data."
        ),
    )
    coverage_service_name_arg = DeclareLaunchArgument(
        "coverage_service_name",
        default_value="/coverage_planner_node/generate_coverage_path",
        description="The running coverage_planner_node's GenerateCoveragePath service.",
    )
    allow_flatness_override_arg = DeclareLaunchArgument(
        "allow_flatness_override",
        default_value="false",
        choices=["true", "false"],
        description=(
            "If the wall exceeds its own plane-fit flatness thresholds, warn instead of refusing "
            "to load it."
        ),
    )
    output_json_arg = DeclareLaunchArgument(
        "output_json",
        default_value="",
        description=(
            "Save this run's coverage-planner output (flat + final path/segments) to this JSON "
            "path for later review. Empty (the default) skips saving."
        ),
    )

    add_strip_connectors_arg = DeclareLaunchArgument(
        "add_strip_connectors",
        default_value="false",
        choices=["true", "false"],
        description=(
            "Interpolate a real connector waypoint chain from one strip's end to the next "
            "strip's start. Default false: only each strip's own two endpoints are planned."
        ),
    )
    allow_unknown_arg = DeclareLaunchArgument(
        "allow_unknown",
        default_value="false",
        choices=["true", "false"],
        description=(
            "Treat cells the wall's own scan never covered as free instead of refusing to plan "
            "through them. Default false (safer)."
        ),
    )
    orientation_format_arg = DeclareLaunchArgument(
        "orientation_format",
        default_value="both",
        choices=["quaternion", "rpy", "both"],
        description=(
            "How output_json writes each waypoint's orientation: 'quaternion', 'rpy' "
            "(roll/pitch/yaw radians), or 'both'."
        ),
    )

    # Tool coverage/orientation params -- see buildToolCoverageProfile()'s own doc comment.
    tool_name_arg = DeclareLaunchArgument("tool_name", default_value="mop")
    contact_arg = DeclareLaunchArgument(
        "contact",
        default_value="true",
        choices=["true", "false"],
        description="True for a tool physically touching the wall (path stays on the surface).",
    )
    standoff_m_arg = DeclareLaunchArgument(
        "standoff_m",
        default_value="0.0",
        description="Stand-off along the wall normal; 0.0 for contact tools.",
    )
    tool_normal_policy_arg = DeclareLaunchArgument(
        "tool_normal_policy",
        default_value="constant",
        choices=["constant", "tilt_at_edges"],
        description='"constant" or "tilt_at_edges" (non-contact tools only).',
    )
    max_edge_tilt_deg_arg = DeclareLaunchArgument(
        "max_edge_tilt_deg", default_value="0.0"
    )
    tool_frame_yaw_deg_arg = DeclareLaunchArgument(
        "tool_frame_yaw_deg", default_value="0.0"
    )
    swath_m_arg = DeclareLaunchArgument(
        "swath_m",
        default_value="0.2",
        description="Effective tool footprint width (m).",
    )
    overlap_pct_arg = DeclareLaunchArgument(
        "overlap_pct",
        default_value="20.0",
        description="step_m = swath_m * (1 - overlap_pct / 100).",
    )
    sweep_direction_arg = DeclareLaunchArgument(
        "sweep_direction",
        default_value="vertical",
        choices=["vertical", "horizontal"],
        description='"vertical" or "horizontal".',
    )
    enable_boundary_pass_arg = DeclareLaunchArgument(
        "enable_boundary_pass", default_value="false", choices=["true", "false"]
    )
    boundary_offset_m_arg = DeclareLaunchArgument(
        "boundary_offset_m", default_value="0.0"
    )
    obstacle_inflation_arg = DeclareLaunchArgument(
        "obstacle_inflation",
        default_value="uniform",
        choices=["uniform", "primary_axis"],
        description='"uniform" or "primary_axis".',
    )
    # obstacle_inflation_radius_m intentionally not exposed here -- wall_coverage_demo_node's own
    # declare_parameter() default (swath_m / 2.0) only applies when the param is left unset;
    # passing any fixed literal through this launch file would defeat that per-run fallback.
    double_pass_arg = DeclareLaunchArgument(
        "double_pass",
        default_value="false",
        choices=["true", "false"],
        description="Two identical-standoff passes, second one reversed (e.g. a mop).",
    )

    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz",
        default_value="true",
        choices=["true", "false"],
        description="Launch RViz to watch the coverage plan.",
    )

    return LaunchDescription(
        [
            wall_scene_json_arg,
            wall_id_arg,
            resolution_arg,
            coverage_service_name_arg,
            allow_flatness_override_arg,
            output_json_arg,
            add_strip_connectors_arg,
            allow_unknown_arg,
            orientation_format_arg,
            tool_name_arg,
            contact_arg,
            standoff_m_arg,
            tool_normal_policy_arg,
            max_edge_tilt_deg_arg,
            tool_frame_yaw_deg_arg,
            swath_m_arg,
            overlap_pct_arg,
            sweep_direction_arg,
            enable_boundary_pass_arg,
            boundary_offset_m_arg,
            obstacle_inflation_arg,
            double_pass_arg,
            show_rviz_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
