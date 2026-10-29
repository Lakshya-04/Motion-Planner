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
    coverage_planner_pcd ${coverage_planner_msgs_TARGETS})
  ament_target_dependencies(wall_coverage_service_node
    rclcpp nav_msgs geometry_msgs std_srvs visualization_msgs)

  add_executable(wall_coverage_demo_node src/ros/wall_coverage_demo_node.cpp)
  target_link_libraries(wall_coverage_demo_node
    coverage_planner_pcd ${coverage_planner_msgs_TARGETS})
  ament_target_dependencies(wall_coverage_demo_node
    rclcpp nav_msgs geometry_msgs sensor_msgs visualization_msgs pcl_conversions)

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
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_add_gtest(test_boustrophedon_planner test/core/boustrophedon_planner_test.cpp)
  target_link_libraries(test_boustrophedon_planner coverage_planner_core)

  ament_add_gtest(test_wall_scene test/core/wall_scene_test.cpp)
  target_link_libraries(test_wall_scene coverage_planner_core)
  target_compile_definitions(test_wall_scene PUBLIC
    WALL_SCENE_SAMPLE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data/wall_scene.json")

  ament_add_gtest(test_wall_pose_lift test/core/wall_pose_lift_test.cpp)
  target_link_libraries(test_wall_pose_lift coverage_planner_core)

  ament_add_gtest(test_coverage_patterns test/core/coverage_patterns_test.cpp)
  target_link_libraries(test_coverage_patterns coverage_planner_core)

  ament_add_gtest(test_sweepline_planner test/core/sweepline_planner_test.cpp)
  target_link_libraries(test_sweepline_planner coverage_planner_core)

  ament_add_gtest(test_coverage_from_scan test/core/coverage_from_scan_test.cpp)
  target_link_libraries(test_coverage_from_scan coverage_planner_core)

  # ros/ tests: message <-> core conversions.
  ament_add_gtest(test_conversions test/ros/conversions_test.cpp)
  target_link_libraries(test_conversions coverage_planner_core ${coverage_planner_msgs_TARGETS})
  ament_target_dependencies(test_conversions nav_msgs geometry_msgs)
endif()

ament_export_targets(export_coverage_planner_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(coverage_planner_msgs core_types scene_model Eigen3 nlohmann_json)
ament_package()
