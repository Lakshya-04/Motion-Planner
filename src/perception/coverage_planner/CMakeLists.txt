cmake_minimum_required(VERSION 3.8)
project(coverage_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(core_types REQUIRED)
find_package(scene_model REQUIRED)

# ------------------------------------------------------------------------------
# core/: ROS-free. Plain C++17 types, interfaces and algorithms. Nothing here may include
# rclcpp or any *_msgs header; the layout is enforced by the include paths below.
# ------------------------------------------------------------------------------
add_library(coverage_planner_core
  src/core/coverage_planner.cpp
  src/core/planners/boustrophedon_planner.cpp
  src/core/planners/sweepline_planner.cpp
  src/core/wall_pose_lift.cpp
  src/core/coverage_pass_expander.cpp
  src/core/wall_scene_loader.cpp
  src/core/wall_rasterizer.cpp
  src/core/coverage_patterns.cpp
  src/core/coverage_from_scan.cpp
  src/core/surface_patch.cpp
  src/core/tool_coverage_profile.cpp
  src/core/coverage_verification.cpp
  src/core/run_scorecard.cpp
  src/core/run_scorecard_io.cpp
  src/core/verify_result.cpp
  src/core/tool_profile_loading.cpp
  src/core/contact_spacing.cpp
)
target_include_directories(coverage_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(coverage_planner_core PUBLIC
  core_types::core_types
  scene_model::scene_model_core
  Eigen3::Eigen
  nlohmann_json::nlohmann_json)
target_compile_features(coverage_planner_core PUBLIC cxx_std_17)

# Optional point-cloud (.pcd) support: the only part of the core that needs PCL.
find_package(PCL QUIET COMPONENTS common io)
if(PCL_FOUND)
  add_library(coverage_planner_pcd src/core/wall_observed_mask_pcd.cpp)
  target_link_libraries(coverage_planner_pcd PUBLIC coverage_planner_core ${PCL_LIBRARIES})
  target_include_directories(coverage_planner_pcd PUBLIC ${PCL_INCLUDE_DIRS})
else()
  message(WARNING "PCL not found: building coverage_planner without .pcd support "
                  "(wall_coverage_service_node and wall_coverage_demo_node are skipped).")
endif()

# ------------------------------------------------------------------------------
# ros/: thin ROS 2 shell (nodes, parameters, message conversions).
# ------------------------------------------------------------------------------
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(coverage_planner_msgs REQUIRED)
find_package(ros_adapters REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(score_run src/core/score_run_main.cpp)
target_link_libraries(score_run coverage_planner_core)
install(TARGETS score_run DESTINATION lib/${PROJECT_NAME})

add_executable(coverage_planner_node src/ros/coverage_planner_node.cpp)
target_link_libraries(coverage_planner_node
  coverage_planner_core
  ros_adapters::ros_adapters
  ${coverage_planner_msgs_TARGETS})
ament_target_dependencies(coverage_planner_node
  rclcpp nav_msgs geometry_msgs std_msgs visualization_msgs)

set(ROS_NODES coverage_planner_node)

if(PCL_FOUND)
  find_package(pcl_conversions REQUIRED)

  add_executable(wall_coverage_service_node src/ros/wall_coverage_service_node.cpp)
  target_link_libraries(wall_coverage_service_node
    coverage_planner_pcd ros_adapters::ros_adapters ${coverage_planner_msgs_TARGETS})
  ament_target_dependencies(wall_coverage_service_node
    rclcpp ament_index_cpp nav_msgs geometry_msgs std_srvs visualization_msgs)

  add_executable(wall_coverage_demo_node src/ros/wall_coverage_demo_node.cpp)
  target_link_libraries(wall_coverage_demo_node
    coverage_planner_pcd ros_adapters::ros_adapters ${coverage_planner_msgs_TARGETS})
  ament_target_dependencies(wall_coverage_demo_node
    rclcpp ament_index_cpp nav_msgs geometry_msgs sensor_msgs visualization_msgs pcl_conversions)

  list(APPEND ROS_NODES wall_coverage_service_node wall_coverage_demo_node)
endif()

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
install(TARGETS ${ROS_NODES} DESTINATION lib/${PROJECT_NAME})

set(CORE_TARGETS coverage_planner_core)
if(PCL_FOUND)
  list(APPEND CORE_TARGETS coverage_planner_pcd)
endif()
install(TARGETS ${CORE_TARGETS}
  EXPORT export_coverage_planner_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY test/data DESTINATION share/${PROJECT_NAME}/test/)
install(PROGRAMS
  scripts/sample_map_publisher.py
  scripts/example_client.py
  scripts/publish_3m_2m_wall.py
  DESTINATION lib/${PROJECT_NAME})

# ------------------------------------------------------------------------------
# Tests: core tests link only the core library, so they run without ROS sourced.
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
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

  ament_add_gtest(test_boustrophedon_planner test/core/boustrophedon_planner_test.cpp)
  target_link_libraries(test_boustrophedon_planner coverage_planner_core)

  ament_add_gtest(test_coverage_components test/core/coverage_components_test.cpp)
  target_link_libraries(test_coverage_components coverage_planner_core)

  ament_add_gtest(test_wall_scene test/core/wall_scene_test.cpp)
  target_link_libraries(test_wall_scene coverage_planner_core)
  target_compile_definitions(test_wall_scene PUBLIC
    WALL_SCENE_SAMPLE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data/wall_scene.json")

  ament_add_gtest(test_wall_pose_lift test/core/wall_pose_lift_test.cpp)
  target_link_libraries(test_wall_pose_lift coverage_planner_core)

  ament_add_gtest(test_coverage_patterns test/core/coverage_patterns_test.cpp)
  target_link_libraries(test_coverage_patterns coverage_planner_core)

  ament_add_gtest(test_edge_spacing_planner test/core/edge_spacing_planner_test.cpp)
  target_link_libraries(test_edge_spacing_planner coverage_planner_core)

  ament_add_gtest(test_sweepline_planner test/core/sweepline_planner_test.cpp)
  target_link_libraries(test_sweepline_planner coverage_planner_core)

  ament_add_gtest(test_coverage_from_scan test/core/coverage_from_scan_test.cpp)
  target_link_libraries(test_coverage_from_scan coverage_planner_core)

  ament_add_gtest(test_surface_patch test/core/surface_patch_test.cpp)
  target_link_libraries(test_surface_patch coverage_planner_core)

  ament_add_gtest(test_coverage_verification test/core/coverage_verification_test.cpp)
  target_link_libraries(test_coverage_verification coverage_planner_core)

  ament_add_gtest(test_spray_deposition test/core/spray_deposition_test.cpp)
  target_link_libraries(test_spray_deposition coverage_planner_core)

  ament_add_gtest(test_run_scorecard test/core/run_scorecard_test.cpp)
  target_link_libraries(test_run_scorecard coverage_planner_core)

  ament_add_gtest(test_verify_result test/core/verify_result_test.cpp)
  target_link_libraries(test_verify_result coverage_planner_core)

  ament_add_gtest(test_run_scorecard_io test/core/run_scorecard_io_test.cpp)
  target_link_libraries(test_run_scorecard_io coverage_planner_core)

  ament_add_gtest(test_contact_spacing test/core/contact_spacing_test.cpp)
  target_link_libraries(test_contact_spacing coverage_planner_core ros_adapters::ros_adapters)
  target_compile_definitions(test_contact_spacing PUBLIC
    TOOL_PROFILES_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/tool_profiles.yaml")

  ament_add_gtest(test_tool_profiles test/core/tool_profiles_test.cpp)
  target_link_libraries(test_tool_profiles coverage_planner_core ros_adapters::ros_adapters)
  target_compile_definitions(test_tool_profiles PUBLIC
    TOOL_PROFILES_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/tool_profiles.yaml")

  # ros/ tests: message <-> core conversions.
  ament_add_gtest(test_conversions test/ros/conversions_test.cpp)
  target_link_libraries(test_conversions coverage_planner_core ${coverage_planner_msgs_TARGETS})
  ament_target_dependencies(test_conversions nav_msgs geometry_msgs)

  # ROS node tests: the node classes run in-process and are driven through their real services.
  # The node sources are compiled into the tests with main() renamed away.
  ament_add_gtest(test_coverage_planner_node test/ros/coverage_planner_node_test.cpp TIMEOUT 120)
  target_link_libraries(test_coverage_planner_node
    coverage_planner_core ros_adapters::ros_adapters ${coverage_planner_msgs_TARGETS})
  ament_target_dependencies(test_coverage_planner_node
    rclcpp nav_msgs geometry_msgs std_msgs visualization_msgs)
  if(PCL_FOUND)
    ament_add_gtest(test_wall_coverage_service_node
      test/ros/wall_coverage_service_node_test.cpp
      TIMEOUT 180)
    target_link_libraries(test_wall_coverage_service_node
      coverage_planner_pcd ros_adapters::ros_adapters ${coverage_planner_msgs_TARGETS})
    ament_target_dependencies(test_wall_coverage_service_node
      rclcpp ament_index_cpp nav_msgs geometry_msgs std_srvs std_msgs visualization_msgs)
    target_compile_definitions(test_wall_coverage_service_node PUBLIC
      WALL_SCENE_SAMPLE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data/wall_scene.json")
  endif()
endif()

ament_export_targets(export_coverage_planner_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(coverage_planner_msgs core_types scene_model Eigen3 nlohmann_json)
ament_package()
