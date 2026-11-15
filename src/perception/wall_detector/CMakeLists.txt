cmake_minimum_required(VERSION 3.8)
project(wall_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(scene_model REQUIRED)

# core/: ROS-free. Plane fit on a point cloud (scene_model) and the wall_scene.json writer.
add_library(wall_detector_core src/core/wall_detection.cpp)
target_include_directories(wall_detector_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(wall_detector_core PUBLIC
  scene_model::scene_model_core Eigen3::Eigen nlohmann_json::nlohmann_json)
target_compile_features(wall_detector_core PUBLIC cxx_std_17)

# ros/: the node (TF, PointCloud2 in, JSON and MoveIt collision object out). Optional.
find_package(rclcpp QUIET)
find_package(sensor_msgs QUIET)
find_package(std_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(moveit_msgs QUIET)
find_package(shape_msgs QUIET)
find_package(tf2_ros QUIET)
find_package(tf2_eigen QUIET)
set(WALL_DETECTOR_TARGETS wall_detector_core)
if(rclcpp_FOUND AND sensor_msgs_FOUND AND std_msgs_FOUND AND geometry_msgs_FOUND AND moveit_msgs_FOUND
   AND shape_msgs_FOUND AND tf2_ros_FOUND AND tf2_eigen_FOUND AND TARGET scene_model::scene_model_ros)
  add_executable(wall_detector_node src/ros/wall_detector_node.cpp)
  target_link_libraries(wall_detector_node wall_detector_core scene_model::scene_model_ros)
  ament_target_dependencies(wall_detector_node
    rclcpp sensor_msgs std_msgs geometry_msgs moveit_msgs shape_msgs tf2_ros tf2_eigen)
  install(TARGETS wall_detector_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
else()
  message(WARNING "rclcpp/tf2/moveit_msgs/scene_model_ros not found: building wall_detector without the node")
endif()

install(TARGETS ${WALL_DETECTOR_TARGETS}
  EXPORT export_wall_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(wall_detection_test test/core/wall_detection_test.cpp)
  target_link_libraries(wall_detection_test wall_detector_core)
endif()

ament_export_targets(export_wall_detector HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 nlohmann_json scene_model)
ament_package()
