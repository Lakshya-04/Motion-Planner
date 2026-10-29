cmake_minimum_required(VERSION 3.8)
project(robot_self_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(scene_model REQUIRED)

# tinyxml2 is a plain system library on every distro (no ROS dependency, no version drift like
# urdfdom 3 -> 4), so find it directly.
find_path(TINYXML2_INCLUDE_DIR tinyxml2.h REQUIRED)
find_library(TINYXML2_LIBRARY tinyxml2 REQUIRED)

# core/: ROS-free. URDF link shapes + kinematics, and the filter geometry.
add_library(robot_self_filter_core
  src/core/robot_model.cpp
  src/core/self_filter.cpp
)
target_include_directories(robot_self_filter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TINYXML2_INCLUDE_DIR})
target_link_libraries(robot_self_filter_core PUBLIC
  scene_model::scene_model_core Eigen3::Eigen ${TINYXML2_LIBRARY})
target_compile_features(robot_self_filter_core PUBLIC cxx_std_17)

# ros/: node + parameters + TF. Optional: the core builds without rclcpp.
find_package(rclcpp QUIET)
find_package(sensor_msgs QUIET)
find_package(std_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(tf2_ros QUIET)
find_package(ament_index_cpp QUIET)
set(SELF_FILTER_TARGETS robot_self_filter_core)
if(rclcpp_FOUND AND sensor_msgs_FOUND AND std_msgs_FOUND AND geometry_msgs_FOUND AND tf2_ros_FOUND
   AND ament_index_cpp_FOUND AND TARGET scene_model::scene_model_ros)
  add_library(robot_self_filter_ros src/ros/self_filter_node.cpp)
  target_link_libraries(robot_self_filter_ros PUBLIC robot_self_filter_core scene_model::scene_model_ros)
  ament_target_dependencies(robot_self_filter_ros PUBLIC
    rclcpp sensor_msgs std_msgs geometry_msgs tf2_ros ament_index_cpp)
  add_executable(self_filter_node src/ros/self_filter_main.cpp)
  target_link_libraries(self_filter_node robot_self_filter_ros)
  install(TARGETS self_filter_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)
  list(APPEND SELF_FILTER_TARGETS robot_self_filter_ros)
else()
  message(WARNING "rclcpp/tf2/scene_model_ros not found: building robot_self_filter without the ROS node")
endif()

install(TARGETS ${SELF_FILTER_TARGETS}
  EXPORT export_robot_self_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(t robot_model self_filter real_robot)
    ament_add_gtest(${t}_test test/core/${t}_test.cpp)
    target_link_libraries(${t}_test robot_self_filter_core)
    target_compile_definitions(${t}_test PRIVATE
      SELF_FILTER_TEST_DATA="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  endforeach()
  if(TARGET robot_self_filter_ros)
    ament_add_gtest(self_filter_node_test test/ros/self_filter_node_test.cpp)
    target_link_libraries(self_filter_node_test robot_self_filter_ros)
  endif()
endif()

ament_export_targets(export_robot_self_filter HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 scene_model)
ament_package()
