cmake_minimum_required(VERSION 3.8)
project(manipulation_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(core_types REQUIRED)
find_package(ros_adapters REQUIRED)
# PCL (and its own Findlibusb.cmake module) must be resolved BEFORE motion_planner --
# motion_planner_pcd's exported link interface references the libusb::libusb imported
# target that only PCL's own find_package() call defines; motion_plannerConfig.cmake
# doesn't re-run it itself.
find_package(PCL REQUIRED COMPONENTS common io)
find_package(motion_planner REQUIRED)
find_package(motion_planner_moveit REQUIRED)
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

add_executable(manipulation_manager_node src/ros/manipulation_manager_node.cpp)
target_compile_features(manipulation_manager_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(manipulation_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(manipulation_manager_node
  rclcpp
  std_msgs
)
# The node is the composition root: it wires the MoveIt/OMPL backend (motion_planner_moveit)
# into the ROS-free core. Swapping the backend (cuRobo, VAMP) means changing only this link
# and the factory in the node.
target_link_libraries(manipulation_manager_node
  ${PROJECT_NAME}_core
  ros_adapters::ros_adapters
  motion_planner_moveit::motion_planner_moveit_lib
)

install(TARGETS
  ${PROJECT_NAME}_core
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

  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    set(ament_cmake_uncrustify_FOUND TRUE)
    set(ament_cmake_flake8_FOUND TRUE)
    set(ament_cmake_pep257_FOUND TRUE)
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
