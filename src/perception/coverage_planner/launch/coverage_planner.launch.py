from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("coverage_planner"),
                "config",
                "coverage_planner.yaml",
            ]
        ),
        description="Path to coverage planner configuration file",
    )

    map_topic_arg = DeclareLaunchArgument(
        "map_topic",
        default_value="/map",
        description="Topic to subscribe for occupancy grid map",
    )

    obstacle_inflation_radius_arg = DeclareLaunchArgument(
        "obstacle_inflation_radius",
        default_value="0.01",
        description="Radius to inflate obstacles in the occupancy grid",
    )
    planning_cpu_set_arg = DeclareLaunchArgument(
        "planning_cpu_set",
        default_value="7-8",
        description="CPU cores to pin coverage planner to",
    )

    # Coverage planner node
    coverage_planner_node = Node(
        package="coverage_planner",
        executable="coverage_planner_node",
        name="coverage_planner_node",
        prefix=["taskset", " -c ", LaunchConfiguration("planning_cpu_set")],
        output="screen",
        parameters=[
            # Service requests with an embedded map take priority over this topic.
            LaunchConfiguration("config_file"),
            {"map_topic": LaunchConfiguration("map_topic")},
            {
                "obstacle_inflation_radius": LaunchConfiguration(
                    "obstacle_inflation_radius"
                )
            },
        ],
        # remappings=[
        #     ('/map', LaunchConfiguration('map_topic')),
        # ]
    )

    return LaunchDescription(
        [
            config_file_arg,
            map_topic_arg,
            obstacle_inflation_radius_arg,
            planning_cpu_set_arg,
            coverage_planner_node,
        ]
    )
