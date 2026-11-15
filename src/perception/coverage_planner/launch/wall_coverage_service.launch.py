import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Fields of the selected tool profile that a launch argument can override (empty = keep profile).
TOOL_OVERRIDES = [
    "contact",
    "standoff_m",
    "tool_normal_policy",
    "max_edge_tilt_deg",
    "tool_frame_yaw_deg",
    "swath_m",
    "overlap_pct",
    "sweep_direction",
    "enable_boundary_pass",
    "boundary_offset_m",
    "obstacle_inflation",
    "double_pass",
]


def launch_setup(context, *args, **kwargs):
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

    node_params = {
        "wall_scene_json": LaunchConfiguration("wall_scene_json"),
        "wall_id": LaunchConfiguration("wall_id"),
        "resolution": LaunchConfiguration("resolution"),
        "add_strip_connectors": LaunchConfiguration("add_strip_connectors"),
        "allow_unknown": LaunchConfiguration("allow_unknown"),
        "tool_name": LaunchConfiguration("tool_name"),
        "tool_profiles_file": LaunchConfiguration("tool_profiles_file"),
    }
    # Per-field overrides of the selected tool profile: only passed when the user set them.
    for name in TOOL_OVERRIDES:
        value = LaunchConfiguration(name).perform(context)
        if value != "":
            node_params[name] = yaml.safe_load(value)

    wall_coverage_service_node = Node(
        package="coverage_planner",
        executable="wall_coverage_service_node",
        name="wall_coverage_service_node",
        output="screen",
        parameters=[node_params],
    )

    return [coverage_planner_node, wall_coverage_service_node]


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

    # Tool selection and overrides. Overrides default to "" = keep the profile's value.
    tool_profiles_file_arg = DeclareLaunchArgument(
        "tool_profiles_file",
        default_value="",
        description="tool_profiles.yaml to read (empty = the file this package ships).",
    )
    tool_name_arg = DeclareLaunchArgument(
        "tool_name",
        default_value="contact_wiper",
        description="Profile in tool_profiles_file: contact_wiper or spray_gun when shipped.",
    )
    override_args = [
        DeclareLaunchArgument(
            name,
            default_value="",
            description=f"Override '{name}' of the tool profile (empty = profile value).",
        )
        for name in TOOL_OVERRIDES
    ]

    return LaunchDescription(
        [
            wall_scene_json_arg,
            wall_id_arg,
            resolution_arg,
            add_strip_connectors_arg,
            allow_unknown_arg,
            tool_profiles_file_arg,
            tool_name_arg,
            *override_args,
            OpaqueFunction(function=launch_setup),
        ]
    )
