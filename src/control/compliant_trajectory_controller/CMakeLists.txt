cmake_minimum_required(VERSION 3.8)
project(compliant_trajectory_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# MoveIt's apt binaries are built against the system octomap (liboctomap1.9), while ros-jazzy-octomap
# ships a newer one that find_package(octomap) would pick first. Two octomap copies in one process
# free the same statics twice at exit (malloc_consolidate abort). Resolve the one MoveIt links.
if(NOT octomap_DIR AND EXISTS /usr/share/octomap/octomap-config.cmake)
  set(octomap_DIR /usr/share/octomap CACHE PATH "octomap package directory (same copy as MoveIt)")
endif()
find_package(ament_cmake REQUIRED)
find_package(admittance_controller REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(kinematics_interface REQUIRED)
find_package(core_types REQUIRED)
find_package(ros_adapters REQUIRED)
find_package(force_core REQUIRED)
find_package(mobile_manipulator_profile REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)

# The real core_types::CompliancePort implementation backing this package's JTAC prototype --
# see its own header doc for why it lives here (platform, an rclcpp/admittance_controller/
# kinematics_interface adapter) rather than in capabilities alongside the port interface
# itself. A real library, not test-only scaffolding: force_core::ForceStageSequencer needs a
# genuine CompliancePort to drive, and the gtest below is what proves this one against real
# AdmittanceRule physics, not a mock standing in for it.
add_library(admittance_rule_compliance_port
  src/admittance_rule_compliance_port.cpp
)
target_include_directories(admittance_rule_compliance_port PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(admittance_rule_compliance_port
  admittance_controller::admittance_controller
  admittance_controller::admittance_controller_parameters
  core_types::core_types
)
ament_target_dependencies(admittance_rule_compliance_port
  controller_interface
  Eigen3
  kinematics_interface
  pluginlib
  rclcpp_lifecycle
  trajectory_msgs
)

# The real controller_manager plugin: replaces joint_trajectory_controller/JointTrajectoryController
# outright for the arm+lift chain, running the same real AdmittanceRule across all 7 joints --
# see jtac_controller.hpp's own doc for why this is the (B) fused design, not (A) chained.
add_library(jtac_controller SHARED
  src/jtac_controller.cpp
)
target_include_directories(jtac_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(jtac_controller
  admittance_controller::admittance_controller
  admittance_controller::admittance_controller_parameters
  joint_trajectory_controller::joint_trajectory_controller
)
ament_target_dependencies(jtac_controller
  control_msgs
  controller_interface
  Eigen3
  hardware_interface
  kinematics_interface
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  trajectory_msgs
)
pluginlib_export_plugin_description_file(controller_interface jtac_controller.xml)

# ROS-client CompliancePort implementation: connects force_core::ForceStageSequencer's
# per-segment engage()/disengage()/setTargetForce() calls to a RUNNING JtacController instance
# over ordinary ROS interfaces (parameters + topics) -- see its own header doc for exactly which
# mechanism backs each CompliancePort method and what is still a disclosed gap.
add_library(jtac_controller_compliance_port
  src/jtac_controller_compliance_port.cpp
)
target_include_directories(jtac_controller_compliance_port PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(jtac_controller_compliance_port
  core_types::core_types
)
ament_target_dependencies(jtac_controller_compliance_port
  control_msgs
  geometry_msgs
  rclcpp
  rclcpp_action
  std_msgs
)

# Live RViz demo of the redundancy split that the admittance-rule integration test proves numerically.
add_executable(jtac_compliance_demo_node
  src/jtac_compliance_demo_node.cpp
)
target_link_libraries(jtac_compliance_demo_node
  admittance_controller::admittance_controller
  admittance_controller::admittance_controller_parameters
  mobile_manipulator_profile::mobile_manipulator_profile
)
ament_target_dependencies(jtac_compliance_demo_node
  ament_index_cpp
  controller_interface
  Eigen3
  geometry_msgs
  kinematics_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  trajectory_msgs
  urdf
  visualization_msgs
)

# Reusable diagnostic: checks a candidate reference pose's Jacobian conditioning before trusting a
# compliance experiment against it. See its own doc comment.
add_executable(pose_conditioning_probe
  src/pose_conditioning_probe.cpp
)
ament_target_dependencies(pose_conditioning_probe
  Eigen3
  kinematics_interface
  pluginlib
  rclcpp
)

# Reusable diagnostic: checks a candidate reference pose for self-collision, using the SAME
# planning_scene::PlanningScene::isStateColliding() call production motion planning uses. See its
# own doc comment.
add_executable(reference_pose_collision_probe
  src/reference_pose_collision_probe.cpp
)
target_link_libraries(reference_pose_collision_probe mobile_manipulator_profile::mobile_manipulator_profile)
target_include_directories(reference_pose_collision_probe PRIVATE include)
ament_target_dependencies(reference_pose_collision_probe
  moveit_core
  moveit_ros_planning
  rclcpp
)

# MoveGroupInterface plan-and-execute test: proves moveit::planning_interface::MoveGroupInterface can plan AND execute a
# real joint-space goal through JtacController -- see the file's own doc comment.
add_executable(moveit_jtac_execution_probe
  src/moveit_jtac_execution_probe.cpp
)
target_link_libraries(moveit_jtac_execution_probe mobile_manipulator_profile::mobile_manipulator_profile)
ament_target_dependencies(moveit_jtac_execution_probe
  moveit_ros_planning_interface
  rclcpp
)

# Reusable diagnostic: plan-only Cartesian-pose-to-joint-solution finder, validated against
# whatever planning scene is currently active. See the file's own doc comment.
add_executable(moveit_pose_finder
  src/moveit_pose_finder.cpp
)
target_link_libraries(moveit_pose_finder mobile_manipulator_profile::mobile_manipulator_profile)
ament_target_dependencies(moveit_pose_finder
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  tf2
  tf2_eigen
)

# Reusable diagnostic: ray-marches the live pushed wall octomap for the first genuinely occupied
# point along a direction -- pure geometry, no IK/robot involved. See the file's own doc comment.
add_executable(wall_surface_probe
  src/wall_surface_probe.cpp
)
target_link_libraries(wall_surface_probe octomap)
ament_target_dependencies(wall_surface_probe
  moveit_msgs
  octomap_msgs
  rclcpp
  tf2_eigen
)

# Reusable octomap+FK ground-truth query -- extracted once it needed a second consumer
# (jtac_wall_approach_test_node.cpp's own post-hoc verification, wall_contact_force_simulator_
# node.cpp's continuous mock wrench). See the header's own doc comment.
add_library(wall_octomap_probe
  src/wall_octomap_probe.cpp
)
target_include_directories(wall_octomap_probe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(wall_octomap_probe octomap)
ament_target_dependencies(wall_octomap_probe
  kinematics_interface
  moveit_msgs
  octomap_msgs
  pluginlib
  rclcpp
  std_msgs
  tf2_eigen
)

# Persistent MOCK contact-force sensor -- see the file's own doc comment for why this exists and
# what it replaces (a one-shot scripted wrench injection with no real contact dynamics at all).
add_executable(wall_contact_force_simulator_node
  src/wall_contact_force_simulator_node.cpp
)
target_link_libraries(wall_contact_force_simulator_node
  wall_octomap_probe
)
ament_target_dependencies(wall_contact_force_simulator_node
  geometry_msgs
  rclcpp
  sensor_msgs
)

# Engage/disengage test: force_core::ForceStageSequencer driven end to end (APPROACH -> CARTESIAN ->
# RETRACT) against JtacControllerCompliancePort, talking over ROS to an ALREADY-RUNNING
# JtacController instance -- see the file's own doc comment for what this proves beyond the sequencer-admittance test.
add_executable(jtac_engage_disengage_test_node
  src/jtac_engage_disengage_test_node.cpp
)
target_link_libraries(jtac_engage_disengage_test_node
  jtac_controller_compliance_port
  force_core::force_core
  core_types::core_types
)
ament_target_dependencies(jtac_engage_disengage_test_node
  control_msgs
  geometry_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
)

# Bridges the engage and wall-approach tests' abstract engage/APPROACH/CARTESIAN/RETRACT cycle and the full production
# pipeline (MoveGroupInterface execute test): the SAME cycle, against a REAL, visible wall -- see the file's own doc comment.
add_executable(jtac_wall_approach_test_node
  src/jtac_wall_approach_test_node.cpp
)
target_link_libraries(jtac_wall_approach_test_node
  jtac_controller_compliance_port
  force_core::force_core
  core_types::core_types
  wall_octomap_probe
  mobile_manipulator_profile::mobile_manipulator_profile
)
target_include_directories(jtac_wall_approach_test_node PRIVATE include)
ament_target_dependencies(jtac_wall_approach_test_node
  control_msgs
  geometry_msgs
  ros_adapters
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  sensor_msgs
  tf2
  visualization_msgs
)

install(TARGETS
  jtac_compliance_demo_node
  jtac_engage_disengage_test_node
  jtac_wall_approach_test_node
  moveit_jtac_execution_probe
  moveit_pose_finder
  pose_conditioning_probe
  reference_pose_collision_probe
  wall_contact_force_simulator_node
  wall_surface_probe
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  admittance_rule_compliance_port
  jtac_controller
  jtac_controller_compliance_port
  wall_octomap_probe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES jtac_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch rviz urdf config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/send_test_trajectory_and_inject_wrench.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(admittance_rule_compliance_port jtac_controller jtac_controller_compliance_port)
ament_export_dependencies(core_types force_core joint_trajectory_controller realtime_tools)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Admittance-rule integration test: proves ros2_control's AdmittanceRule can be driven from OUTSIDE its own controller,
  # against a mixed prismatic+revolute 7-joint chain. This is the load-bearing assumption behind
  # building a fused controller at all.
  #
  # Note admittance_rule.hpp does NOT include admittance_rule_impl.hpp: the implementation is
  # compiled into libadmittance_controller.so, and all 8 AdmittanceRule symbols are exported from
  # it. So we link the maintained upstream library rather than vendoring or recompiling the impl.
  ament_add_gtest(test_admittance_rule_integration
    test/test_admittance_rule_integration.cpp
  )
  if(TARGET test_admittance_rule_integration)
    target_link_libraries(test_admittance_rule_integration
      admittance_controller::admittance_controller
      admittance_controller::admittance_controller_parameters
    )
    ament_target_dependencies(test_admittance_rule_integration
      controller_interface
      geometry_msgs
      kinematics_interface
      pluginlib
      rclcpp
      rclcpp_lifecycle
      trajectory_msgs
      control_msgs
      Eigen3
    )
  endif()

  # Sequencer-over-admittance-port test: force_core::ForceStageSequencer/ForceStageFsm driven end to end (APPROACH ->
  # CARTESIAN -> RETRACT) against AdmittanceRuleCompliancePort's real AdmittanceRule physics. See
  # the test file's own doc comment for why this needs no ExecutionPort/ManipulationManager.
  ament_add_gtest(test_force_stage_sequencer_admittance_integration
    test/test_force_stage_sequencer_admittance_integration.cpp
  )
  if(TARGET test_force_stage_sequencer_admittance_integration)
    target_link_libraries(test_force_stage_sequencer_admittance_integration
      admittance_rule_compliance_port
      force_core::force_core
      core_types::core_types
    )
    ament_target_dependencies(test_force_stage_sequencer_admittance_integration
      Eigen3
      kinematics_interface
      pluginlib
      rclcpp
    )
  endif()

  # Redundancy-weighting test: pins weightedPseudoInverse() (JtacController's per-joint redundancy weighting) --
  # see the test file's own doc comment for why this tests the extracted pure-math function in
  # isolation rather than JtacController::updateWeighted()'s surrounding frame-rotation logic,
  # which is verified live instead.
  ament_add_gtest(test_redundancy_weighting
    test/test_redundancy_weighting.cpp
  )
  if(TARGET test_redundancy_weighting)
    target_link_libraries(test_redundancy_weighting
      jtac_controller
    )
    ament_target_dependencies(test_redundancy_weighting
      Eigen3
    )
  endif()

  # Input checks of the probe / wall-approach nodes (no-lift profile, failed load, other arm).
  ament_add_gtest(test_pose_inputs
    test/test_pose_inputs.cpp
  )
  if(TARGET test_pose_inputs)
    target_link_libraries(test_pose_inputs
      mobile_manipulator_profile::mobile_manipulator_profile
    )
    target_include_directories(test_pose_inputs PRIVATE include)
  endif()
endif()

ament_package()
