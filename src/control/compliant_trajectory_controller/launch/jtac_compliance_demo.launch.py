"""Launch the JTAC compliance demo: the demo node plus two robot_state_publisher instances
(actual + a "ghost_"-prefixed reference overlay) and RViz, so the redundancy split proven in
test_admittance_rule_integration.cpp (T4a) can actually be watched rather than read off numbers.

Defaults to this repo's real description chain (urdf/robot.urdf.xacro, base_type:=serum), not
the synthetic T4a stand-in -- pass real_robot:=false to fall back to the small self-contained demo
geometry. Ported from an earlier internal repo, where this rendered robot_description's own
robot.urdf.xacro -- see jtac_controller_test.launch.py's own header for why that package isn't
in this repo and how the render was adapted.

Usage:
    ros2 launch compliant_trajectory_controller jtac_compliance_demo.launch.py
    ros2 launch compliant_trajectory_controller jtac_compliance_demo.launch.py real_robot:=false
    ros2 launch compliant_trajectory_controller jtac_compliance_demo.launch.py show_rviz:=false
    ros2 launch compliant_trajectory_controller jtac_compliance_demo.launch.py pose_set:=wall_ceiling
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Vetted with pose_conditioning_probe AND reference_pose_collision_probe (this package) against
# THIS EXACT rendering -- base_type:=serum, attach_rpy:="0 0 0.7854", mop tool geometry attached
# -- not the plain-default render, and checked with kinematics.tip=link_tcp (the real controlled
# point once the tool is attached, not the bare link6 flange). A pose checked only for Jacobian
# conditioning, or against a different base_type/attach_rpy/tip, is not validated for this robot.
#
# An earlier candidate (elbow bent the "wrong" way: joint3=1.2) was NEVER PHYSICALLY REACHABLE --
# joint3's real limit is [-3.11, 0.061] -- and slipped through undetected because neither probe
# checked joint limits at the time (fixed in reference_pose_collision_probe.cpp; it now refuses
# any out-of-bounds pose outright). A reachable, more genuinely "arm reaching toward a wall"
# extended pose (elbow only slightly bent, condition number 17.5) was found by sweeping poses
# within the real joint limits and confirmed collision-free with 3x the compliance offset margin
# actually observed live, in both push directions.
REAL_ROBOT_REFERENCE_POSE = [0.40, 0.0, -0.15, -0.15, 0.0, -0.15, 0.0]
# Matches jtac_demo_robot.urdf's own bent, non-singular pose (see that file's own comment).
SYNTHETIC_REFERENCE_POSE = [0.40, 0.0, 0.6, -0.9, 0.0, 0.5, 0.0]


def _render_real_robot_description(context):
    """Renders description/urdf/robot.urdf.xacro (base_type:=serum, the default -- lift +
    UF850 arm + mop tool, matching jtac_controller_test.launch.py's own real-robot render) with
    arm_control_mode:=joint_trajectory (that xacro's own non-production joint-command interface --
    see its header) so this demo's KDL kinematics has real joint state to work from.
    use_mock_hardware:=true keeps this purely kinematic; no hardware ports touched.
    """
    pkg_desc_dir = get_package_share_directory("description")
    xacro_path = os.path.join(pkg_desc_dir, "urdf", "robot.urdf.xacro")
    command = [
        "xacro",
        xacro_path,
        "base_type:=serum",
        "use_mock_hardware:=true",
        "arm_control_mode:=joint_trajectory",
    ]
    return subprocess.run(command, check=True, capture_output=True, text=True).stdout


def _launch_setup(context, *args, **kwargs):
    real_robot = LaunchConfiguration("real_robot").perform(context) == "true"
    show_rviz = LaunchConfiguration("show_rviz")
    pkg_share = get_package_share_directory("compliant_trajectory_controller")
    rviz_config_path = os.path.join(pkg_share, "rviz", "jtac_compliance_demo.rviz")

    if real_robot:
        robot_description = _render_real_robot_description(context)
        reference_pose = REAL_ROBOT_REFERENCE_POSE
        # link_tcp, not link6: the real chain is rendered with add_other_geometry:=true (the
        # tool from the robot profile), so link6 is the bare wrist flange,
        # NOT the controlled tool point -- the repo's tool-point convention (controlled tool point:
        # link_tcp; the bare flange is not the tool tip, see docs/design_principles.md). Using link6
        # here means admittance has no awareness of the tool geometry swinging further out on
        # every correction, which is exactly what let a real self-collision through undetected.
        kinematics_tip = "link_tcp"
    else:
        demo_urdf_path = os.path.join(pkg_share, "urdf", "jtac_demo_robot.urdf")
        with open(demo_urdf_path, "r") as urdf_file:
            robot_description = urdf_file.read()
        reference_pose = SYNTHETIC_REFERENCE_POSE
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
                "reference_pose": reference_pose,
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
                description="true: render the real robot_description chain "
                "(robot.urdf.xacro, base_type:=serum, the real attach_rpy, the "
                "wall_cleaning_default tool geometry -- matching cartesian_test.launch.py). "
                "false: use the small synthetic T4a-style stand-in geometry instead.",
            ),
            DeclareLaunchArgument(
                "pose_set",
                default_value="redundancy_split",
                description="'redundancy_split' (default): the original demo, base-frame axis "
                "pushes at REAL_ROBOT_REFERENCE_POSE/SYNTHETIC_REFERENCE_POSE. 'wall_ceiling': "
                "tool-local-frame X/Y/Z pushes at a real, validated HOME/WALL pose (the same "
                "joint3=-25deg/joint5=65deg seed workflow_orchestrator's "
                "validate_7dof_sync.py uses), then the same at a raised CEILING pose -- requires "
                "real_robot:=true (its default). See jtac_compliance_demo_node.cpp's "
                "kHomeWallPose/kCeilingPose for how each was vetted.",
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
