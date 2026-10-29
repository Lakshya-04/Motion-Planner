cmake_minimum_required(VERSION 3.8)
project(coverage_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(coverage_planner_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)

# Generate this package's own message/service types -- no dependency on any sibling *
# package; WallPlane/ObstacleRegion/MotionPlanSegment are built entirely from standard
# nav_msgs/geometry_msgs field types, the same way GenerateCoveragePath.srv itself already
# reuses nav_msgs/OccupancyGrid, nav_msgs/Path, geometry_msgs/PolygonStamped.
# Message/service headers are generated into this package's own build tree; sources below
# include e.g. "coverage_planner_msgs/msg/wall_plane.hpp" and must see it before install-time
# resolution would otherwise provide it.

# Core loader/rasterizer library -- exported so packages outside this one (e.g.
# serum_demo_bt's fixture-map publisher) can reuse loadWallScene()/
# rasterizeWallCoverageGrid() instead of redeclaring the same wall-scene-loading logic.
# coverage_planner_node/wall_coverage_demo_node below keep compiling these sources directly
# rather than linking this (pre-existing pattern, unchanged here) -- this library only exists
# for external consumers.
add_library(coverage_planner_core
  src/wall_scene_loader.cpp
  src/wall_rasterizer.cpp
  src/wall_pose_lift.cpp
  src/coverage_pass_expander.cpp
)

ament_target_dependencies(coverage_planner_core
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
)

target_link_libraries(coverage_planner_core
  ${coverage_planner_msgs_TARGETS}
  nlohmann_json::nlohmann_json
  ${PCL_LIBRARIES}
)

target_include_directories(coverage_planner_core PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
  $<BUILD_INTERFACE:${PCL_INCLUDE_DIRS}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(coverage_planner_core PUBLIC cxx_std_17)

# Coverage Planner Node
add_executable(coverage_planner_node
  src/coverage_planner_node.cpp
  src/coverage_planner.cpp
  src/planners/boustrophedon_planner.cpp
  src/wall_scene_loader.cpp
  src/wall_rasterizer.cpp
  src/wall_pose_lift.cpp
  src/coverage_pass_expander.cpp
)

ament_target_dependencies(coverage_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
)

target_link_libraries(coverage_planner_node
  ${coverage_planner_msgs_TARGETS}
  nlohmann_json::nlohmann_json
  ${PCL_LIBRARIES}
)

target_include_directories(coverage_planner_node PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(coverage_planner_node PUBLIC c_std_99 cxx_std_17)

# Wall Coverage Demo Node -- the PCD-in/waypoints-out unit: wall-scene loader -> rasterizer ->
# GenerateCoveragePath -> pose lift + pass expansion, see wall_coverage_demo_node.cpp's doc
# comment. No MoveIt/motion_planner dependency -- reachability/execution against a real
# robot is a separate concern.
add_executable(wall_coverage_demo_node
  src/wall_coverage_demo_node.cpp
  src/wall_scene_loader.cpp
  src/wall_rasterizer.cpp
  src/wall_pose_lift.cpp
  src/coverage_pass_expander.cpp
)

ament_target_dependencies(wall_coverage_demo_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
)

target_link_libraries(wall_coverage_demo_node
  ${coverage_planner_msgs_TARGETS}
  nlohmann_json::nlohmann_json
  ${PCL_LIBRARIES}
)

target_include_directories(wall_coverage_demo_node PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(wall_coverage_demo_node PUBLIC c_std_99 cxx_std_17)

# Wall Coverage Service Node -- wall_coverage_demo_node's own pipeline, standing and
# trigger-controlled instead of one-shot: ~/start (std_srvs/Trigger) kicks it off, the resulting
# plan is published on ~/coverage_plan instead of returned synchronously. See
# wall_coverage_service_node.hpp's own doc comment for why.
add_executable(wall_coverage_service_node
  src/wall_coverage_service_node.cpp
  src/wall_scene_loader.cpp
  src/wall_rasterizer.cpp
  src/wall_pose_lift.cpp
  src/coverage_pass_expander.cpp
)

ament_target_dependencies(wall_coverage_service_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_srvs
  visualization_msgs
)

target_link_libraries(wall_coverage_service_node
  ${coverage_planner_msgs_TARGETS}
  nlohmann_json::nlohmann_json
  ${PCL_LIBRARIES}
)

target_include_directories(wall_coverage_service_node PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(wall_coverage_service_node PUBLIC c_std_99 cxx_std_17)

# Install
install(TARGETS coverage_planner_node wall_coverage_demo_node wall_coverage_service_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS coverage_planner_core
  EXPORT export_coverage_planner_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# test/data: not just gtest fixture data (WALL_SCENE_SAMPLE_PATH reads it from the source tree at
# compile time) -- also wall_coverage_demo.launch.py's own default wall_scene_json, so it must be
# installed for that default to resolve via FindPackageShare in an installed workspace.
install(DIRECTORY
  test/data
  DESTINATION share/${PROJECT_NAME}/test/
)

install(PROGRAMS
  scripts/sample_map_publisher.py
  scripts/example_client.py
  scripts/publish_3m_2m_wall.py
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_boustrophedon_planner
    test/boustrophedon_planner_test.cpp
    src/coverage_planner.cpp
    src/planners/boustrophedon_planner.cpp
  )
  if(TARGET test_boustrophedon_planner)
    ament_target_dependencies(test_boustrophedon_planner
      rclcpp
      nav_msgs
      geometry_msgs
    )
    target_link_libraries(test_boustrophedon_planner ${coverage_planner_msgs_TARGETS})
    target_include_directories(test_boustrophedon_planner PUBLIC
      ${EIGEN3_INCLUDE_DIRS}
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_compile_features(test_boustrophedon_planner PUBLIC cxx_std_17)
  endif()

  ament_add_gtest(test_wall_scene
    test/wall_scene_test.cpp
    src/wall_scene_loader.cpp
    src/wall_rasterizer.cpp
  )
  if(TARGET test_wall_scene)
    ament_target_dependencies(test_wall_scene
      nav_msgs
      geometry_msgs
    )
    target_link_libraries(test_wall_scene
      ${coverage_planner_msgs_TARGETS}
      nlohmann_json::nlohmann_json
      ${PCL_LIBRARIES}
    )
    target_include_directories(test_wall_scene PUBLIC
      ${EIGEN3_INCLUDE_DIRS}
      ${PCL_INCLUDE_DIRS}
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_compile_definitions(test_wall_scene PUBLIC
      WALL_SCENE_SAMPLE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data/wall_scene.json"
    )
    target_compile_features(test_wall_scene PUBLIC cxx_std_17)
  endif()

  ament_add_gtest(test_wall_pose_lift
    test/wall_pose_lift_test.cpp
    src/wall_pose_lift.cpp
    src/coverage_pass_expander.cpp
  )
  if(TARGET test_wall_pose_lift)
    ament_target_dependencies(test_wall_pose_lift
      nav_msgs
      geometry_msgs
    )
    target_link_libraries(test_wall_pose_lift ${coverage_planner_msgs_TARGETS})
    target_include_directories(test_wall_pose_lift PUBLIC
      ${EIGEN3_INCLUDE_DIRS}
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_compile_features(test_wall_pose_lift PUBLIC cxx_std_17)
  endif()
endif()

ament_export_targets(export_coverage_planner_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
# coverage_planner_core links PCL and nlohmann_json and exposes Eigen3/PCL headers on its own
# PUBLIC include dirs (see target_link_libraries/target_include_directories above) -- exporting
# them here means a consumer's own find_package(coverage_planner) resolves those transitively,
# instead of needing its own copy-pasted find_package() calls to match.
ament_export_dependencies(
  coverage_planner_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  Eigen3
  PCL
  nlohmann_json
)
ament_package()
