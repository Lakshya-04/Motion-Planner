"""Launch the JTAC compliance demo: the demo node plus two robot_state_publisher instances
(actual + a "ghost/"-prefixed reference overlay) and RViz, so the redundancy split proven in
test_admittance_rule_integration.cpp (T4a) can actually be watched rather than read off numbers.

By default the robot is rendered from a robot profile through mobile_manipulator_description
(`profile:=` or `arm:=`, default ur10e_telescopic; hardware mode jtac_mock, so it stays purely
kinematic), and the demo node takes the controlled tip and the home pose from that profile. Pass
real_robot:=false to fall back to the small self-contained demo geometry (jtac_demo_robot.urdf).
pose_set:=wall_ceiling is vetted for one 7-joint chain whose tip is named link_tcp; the node
refuses to start otherwise (see its argument description).

Usage:
    ros2 launch compliant_trajectory_controller jtac_compliance_demo.launch.py
    ros2 launch compliant_trajectory_controller jtac_compliance_demo.launch.py profile:=panda_telescopic
    ros2 launch compliant_trajectory_controller jtac_compliance_demo.launch.py real_robot:=false
    ros2 launch compliant_trajectory_controller jtac_compliance_demo.launch.py show_rviz:=false
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mobile_manipulator_description import DEFAULT_PROFILE, load_profile, profile_for_arm


# Matches jtac_demo_robot.urdf's own bent, non-singular pose (see that file's own comment).
SYNTHETIC_REFERENCE_POSE = [0.40, 0.0, 0.6, -0.9, 0.0, 0.5, 0.0]


def _profile_robot(context):
    """(robot_description xml, profile name, tcp link) for the requested profile/arm."""
    arm = LaunchConfiguration("arm").perform(context) or None
    profile = LaunchConfiguration("profile").perform(context) or None
    resolved = load_profile(profile or (profile_for_arm(arm) if arm else DEFAULT_PROFILE))
    xacro_path = os.path.join(
        get_package_share_directory("mobile_manipulator_description"),
        "urdf", "mobile_manipulator.urdf.xacro")
    mappings = dict(resolved.xacro_mappings(), profile=resolved.name, hardware="jtac_mock")
    if arm:
        mappings["arm"] = arm
    robot_description = xacro.process_file(xacro_path, mappings=mappings).toxml()
    return robot_description, resolved.name, resolved.tcp_link


def _launch_setup(context, *args, **kwargs):
    real_robot = LaunchConfiguration("real_robot").perform(context) == "true"
    show_rviz = LaunchConfiguration("show_rviz")
    pkg_share = get_package_share_directory("compliant_trajectory_controller")
    rviz_config_path = os.path.join(pkg_share, "rviz", "jtac_compliance_demo.rviz")

    node_params = {}
    if real_robot:
        robot_description, profile_name, kinematics_tip = _profile_robot(context)
        # The tcp link is the controlled tool point; the bare flange is not the tool tip. The
        # reference pose is left unset so the node uses the profile's home pose.
        node_params["profile"] = profile_name
    else:
        demo_urdf_path = os.path.join(pkg_share, "urdf", "jtac_demo_robot.urdf")
        with open(demo_urdf_path, "r") as urdf_file:
            robot_description = urdf_file.read()
        node_params["reference_pose"] = SYNTHETIC_REFERENCE_POSE
        kinematics_tip = "link6"

    # Comma-separated string, not a repeated launch arg -- ros2 launch has no native list-typed
    # CLI argument, and this matches how other float-list overrides are passed on the command
    # line elsewhere in this repo (parsed back into a real list here, not left as a string for
    # the node to parse itself).
    redundancy_weights = [
        float(w) for w in LaunchConfiguration("redundancy_weights").perform(context).split(",")
    ]

    demo_node = Node(
        package="compliant_trajectory_controller",
        executable="jtac_compliance_demo_node",
        name="jtac_compliance_demo_node",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                **node_params,
                "kinematics_tip": kinematics_tip,
                "pose_set": LaunchConfiguration("pose_set"),
                "enable_redundancy_weighting": LaunchConfiguration("enable_redundancy_weighting"),
                "redundancy_weights": redundancy_weights,
            }
        ],
    )

    # ACTUAL robot: the compliance-shifted pose, at full opacity, unprefixed TF -- this is "the
    # real robot" in the visualization.
    actual_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="actual_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        remappings=[("joint_states", "/joint_states")],
    )

    # GHOST robot: the fixed reference pose the trajectory backbone commanded, frame_prefix'd so
    # its whole TF tree renders under ghost/* frame ids without colliding with the actual robot's.
    # See jtac_compliance_demo_node.cpp's own doc comment for why the reference never moves during
    # a push -- that is the whole point being visualized (compliance is a DELTA on top of it).
    #
    # frame_prefix is "ghost/" WITH the trailing slash (robot_state_publisher concatenates it
    # directly onto each link name: "ghost/" + "base_link" = "ghost/base_link"), while the RViz
    # RobotModel display's own "TF Prefix" field below is bare "ghost" (RViz inserts its own "/"
    # when joining prefix + link name). Mismatching these two conventions -- e.g. "ghost_" on both
    # sides -- silently breaks the ghost overlay: the robot_description topic still connects fine
    # (TF-independent), so nothing errors, but RViz looks up "ghost_/link6" while the broadcast
    # frame is actually "ghost_link6", finds no transform, and renders nothing. This exact
    # "prefix/" + bare-prefix pairing is the one already proven working in this repo -- see
    # PlannerBenchmark's start-state-ghost feature (fsp_test.launch.py, frame_prefix: "start/",
    # benchmark.rviz's TF Prefix: start), reused here rather than re-deriving it.
    ghost_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="ghost_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "frame_prefix": "ghost/"}],
        remappings=[
            ("joint_states", "/ghost/joint_states"),
            # Both instances load the same URDF; remapped so the two /robot_description
            # publishers (one per robot_state_publisher, both latched std_msgs/String) don't
            # collide -- RViz's two RobotModel displays subscribe to these explicitly.
            ("robot_description", "/ghost/robot_description"),
        ],
    )

    # frame_prefix makes robot_state_publisher publish its OWN root link ("ghost/base_link")
    # disconnected from the actual robot's "base_link" -- tie them together at the same physical
    # origin (zero transform) so both overlay in the same view instead of floating apart.
    ghost_root_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="ghost_root_anchor",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "ghost/base_link"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(show_rviz),
        output="screen",
    )

    return [demo_node, actual_state_publisher, ghost_state_publisher, ghost_root_link, rviz_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "real_robot",
                default_value="true",
                description="true: render the robot from the profile (profile:=/arm:=). "
                "false: use the small synthetic T4a-style stand-in geometry instead.",
            ),
            DeclareLaunchArgument(
                "profile", default_value="",
                description="robot profile name or path (default ur10e_telescopic); "
                "real_robot:=true only."),
            DeclareLaunchArgument(
                "arm", default_value="",
                description="arm name; picks the matching profile when `profile` is empty."),
            DeclareLaunchArgument(
                "pose_set",
                default_value="redundancy_split",
                description="'redundancy_split' (default): the original demo, base-frame axis "
                "pushes at the profile's home pose "
                "(SYNTHETIC_REFERENCE_POSE with real_robot:=false). 'wall_ceiling': tool-local "
                "X/Y/Z pushes at a wall pose, then at a raised ceiling pose; the node only "
                "accepts it on the 7-joint chain whose tip link is named link_tcp, so with "
                "profile tips (tcp_link) it refuses to start. See "
                "jtac_compliance_demo_node.cpp's kHomeWallPose/kCeilingPose.",
            ),
            DeclareLaunchArgument(
                "enable_redundancy_weighting",
                default_value="false",
                description="pose_set:=wall_ceiling only. true: replace the plain "
                "AdmittanceRule-driven compliance loop with a self-contained weighted-"
                "pseudo-inverse one (see jtac_compliance_demo_node.cpp's "
                "tickWallCeilingWeighted()) so redundancy_weights actually has an effect. "
                "Default false: unchanged AdmittanceRule-driven behavior.",
            ),
            DeclareLaunchArgument(
                "redundancy_weights",
                default_value="10.0,1.0,1.0,1.0,1.0,1.0,1.0",
                description="enable_redundancy_weighting:=true only. Comma-separated, 7 values "
                "(lift, joint1..6) -- a LARGER weight makes that joint move LESS for the same "
                "Cartesian push; smaller moves more. Default (lift 10x each arm joint) is a "
                "deliberate, reversible design choice -- 'arm more compliant than the lift' -- "
                "not a physically-derived number; see jtac_compliance_demo_node.cpp's own "
                "redundancy_weights doc comment for the live-verified range this sits inside. "
                "All-1.0 reduces to the plain Moore-Penrose pseudo-inverse, reproducing the "
                "unweighted behavior exactly.",
            ),
            DeclareLaunchArgument(
                "show_rviz",
                default_value="true",
                description="Launch RViz alongside the demo (per this repo's convention: verify "
                "live, not headless).",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
