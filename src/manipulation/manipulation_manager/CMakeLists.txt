cmake_minimum_required(VERSION 3.8)
project(manipulation_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(reachability_msgs REQUIRED)
find_package(stand_navigator_msgs REQUIRED)
find_package(coverage_task_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(core_types REQUIRED)
find_package(ros_adapters REQUIRED)
# PCL (and its own Findlibusb.cmake module) must be resolved BEFORE motion_planner --
# motion_planner_pcd's exported link interface references the libusb::libusb imported
# target that only PCL's own find_package() call defines; motion_plannerConfig.cmake
# doesn't re-run it itself.
find_package(PCL REQUIRED COMPONENTS common io)
find_package(motion_planner REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(motion_planner_moveit REQUIRED)  # only the tests still link the MoveIt adapter directly
find_package(mobile_manipulator_profile REQUIRED)
find_package(nlohmann_json REQUIRED)

# core/ (zero ROS deps -- verified: only stdlib + core_types/motion_planner headers)
# and ros/ (JtacExecutionAdapter, CoverageOutputJsonSource) build into one library, mirroring
# motion_planner's own _core-library-plus-executable pattern. sii's original port of
# this package also had a MoveItExecutionAdapter/YAML-file CoverageSource and a benchmark/
# validator/action-server front-end -- none of that applies here (JTAC drives execution, not
# MoveGroupInterface::execute(); coverage comes from coverage_planner's own JSON output,
# not a hardcoded YAML file) or has landed upstream to begin with, so this package only carries
# the core orchestrator plus the two adapters this repo's own backend actually needs.
add_library(${PROJECT_NAME}_core
  src/core/manipulation_manager.cpp
  src/core/motion_exec_reporter.cpp
  src/core/coverage_feasibility_analyzer.cpp
  src/core/base_stance_check.cpp
  src/core/coverage_task.cpp
  src/ros/execution/jtac_execution_adapter.cpp
  src/ros/coverage/coverage_output_json_source.cpp
)

target_link_libraries(${PROJECT_NAME}_core
  core_types::core_types
  ros_adapters::ros_adapters
  motion_planner::motion_planner_core
  nlohmann_json::nlohmann_json
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME}_core PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  rclcpp_action
  control_msgs
)

# Opt-in retiming (retime:=true): the one place the node touches the MoveIt adapter.
add_library(${PROJECT_NAME}_retiming src/ros/execution/retiming_wiring.cpp)
target_link_libraries(${PROJECT_NAME}_retiming PUBLIC ${PROJECT_NAME}_core
  motion_planner_moveit::motion_planner_moveit_lib)
ament_target_dependencies(${PROJECT_NAME}_retiming PUBLIC rclcpp moveit_ros_planning)

add_executable(manipulation_manager_node src/ros/manipulation_manager_node.cpp)
target_compile_features(manipulation_manager_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(manipulation_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(manipulation_manager_node
  rclcpp
  reachability_msgs
  stand_navigator_msgs
  coverage_task_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  ament_index_cpp
  tf2
  tf2_ros
)
# The node is the composition root, but it no longer links a backend: the planners are built
# from the planning_config YAML through pluginlib (motion_planner_ros), so switching backends
# is a config change. The plugin packages (motion_planner_moveit, curobo_planner, vamp_planner)
# only need to be installed.
target_link_libraries(manipulation_manager_node
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_retiming
  ros_adapters::ros_adapters
  motion_planner::motion_planner_ros
  mobile_manipulator_profile::mobile_manipulator_profile
)

install(TARGETS
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_retiming
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  manipulation_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(manipulation_manager_test
    test/manipulation_manager_test.cpp
  )
  if(TARGET manipulation_manager_test)
    target_link_libraries(manipulation_manager_test ${PROJECT_NAME}_core core_types::core_types
      motion_planner_moveit::motion_planner_moveit_lib)
  endif()

  ament_add_gtest(coverage_feasibility_analyzer_test
    test/coverage_feasibility_analyzer_test.cpp
  )
  if(TARGET coverage_feasibility_analyzer_test)
    target_link_libraries(coverage_feasibility_analyzer_test ${PROJECT_NAME}_core core_types::core_types)
  endif()

  ament_add_gtest(base_stance_check_test test/base_stance_check_test.cpp)
  if(TARGET base_stance_check_test)
    target_link_libraries(base_stance_check_test ${PROJECT_NAME}_core core_types::core_types)
  endif()

  ament_add_gtest(coverage_output_json_source_test test/coverage_output_json_source_test.cpp)
  if(TARGET coverage_output_json_source_test)
    target_link_libraries(coverage_output_json_source_test ${PROJECT_NAME}_core core_types::core_types)
  endif()

  ament_add_gtest(wall_direction_test test/wall_direction_test.cpp)
  if(TARGET wall_direction_test)
    target_link_libraries(wall_direction_test ${PROJECT_NAME}_core core_types::core_types)
  endif()

  ament_add_gtest(travel_pose_test test/travel_pose_test.cpp)
  if(TARGET travel_pose_test)
    target_link_libraries(travel_pose_test ${PROJECT_NAME}_core core_types::core_types)
  endif()

  ament_add_gtest(joint_names_test test/joint_names_test.cpp)
  if(TARGET joint_names_test)
    target_link_libraries(joint_names_test ${PROJECT_NAME}_core core_types::core_types)
  endif()

  ament_add_gtest(coverage_task_test test/coverage_task_test.cpp)
  if(TARGET coverage_task_test)
    target_link_libraries(coverage_task_test ${PROJECT_NAME}_core core_types::core_types)
  endif()

  ament_add_gtest(tool_mode_test test/tool_mode_test.cpp)
  if(TARGET tool_mode_test)
    target_include_directories(tool_mode_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()

  ament_add_gtest(frame_transform_test test/frame_transform_test.cpp)
  if(TARGET frame_transform_test)
    target_link_libraries(frame_transform_test ${PROJECT_NAME}_core core_types::core_types)
  endif()

  ament_add_gtest(planning_config_test test/planning_config_test.cpp)
  if(TARGET planning_config_test)
    target_link_libraries(planning_config_test motion_planner::motion_planner_ros)
    target_compile_definitions(planning_config_test PRIVATE
      PLANNING_YAML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/planning.yaml")
  endif()

  ament_add_gtest(recovery_jog_test test/recovery_jog_test.cpp)
  if(TARGET recovery_jog_test)
    target_include_directories(recovery_jog_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()

  ament_add_gtest(column_ordering_test test/column_ordering_test.cpp)
  if(TARGET column_ordering_test)
    target_include_directories(column_ordering_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()

  ament_add_gtest(retiming_execution_port_test test/retiming_execution_port_test.cpp)
  if(TARGET retiming_execution_port_test)
    target_link_libraries(retiming_execution_port_test ${PROJECT_NAME}_core core_types::core_types)
  endif()

  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    set(ament_cmake_uncrustify_FOUND TRUE)
    set(ament_cmake_flake8_FOUND TRUE)
    set(ament_cmake_pep257_FOUND TRUE)
    set(ament_cmake_xmllint_FOUND TRUE)
    set(ament_cmake_lint_cmake_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

ament_export_dependencies(
  rclcpp
  rclcpp_action
  control_msgs
  core_types
  ros_adapters
  motion_planner
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)

ament_package()
