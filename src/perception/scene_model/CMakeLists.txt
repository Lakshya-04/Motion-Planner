cmake_minimum_required(VERSION 3.8)
project(scene_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Linked into the cuRobo plugin shared library (pluginlib), so it must be PIC.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# core/: ROS-free. The scene types, the SceneModel container, file loaders (PCD, PLY, OBJ, STL,
# YAML), voxelization + ESDF, plane fitting, and exporters for the planning backends.
add_library(scene_model_core
  src/core/scene_model.cpp
  src/core/io_pointcloud.cpp
  src/core/io_mesh.cpp
  src/core/io_scene.cpp
  src/core/voxel.cpp
  src/core/plane_fit.cpp
  src/core/export.cpp
)
target_include_directories(scene_model_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(scene_model_core PUBLIC Eigen3::Eigen yaml-cpp)
target_compile_features(scene_model_core PUBLIC cxx_std_17)

# ros/: message conversions (PointCloud2, MoveIt collision objects / planning scene). Messages only,
# no rclcpp. Optional: the core builds without any ROS message package.
find_package(sensor_msgs QUIET)
find_package(shape_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(moveit_msgs QUIET)
set(SCENE_MODEL_TARGETS scene_model_core)
if(sensor_msgs_FOUND AND shape_msgs_FOUND AND geometry_msgs_FOUND AND moveit_msgs_FOUND)
  add_library(scene_model_ros src/ros/conversions.cpp)
  target_link_libraries(scene_model_ros PUBLIC scene_model_core)
  ament_target_dependencies(scene_model_ros PUBLIC sensor_msgs shape_msgs geometry_msgs moveit_msgs)
  list(APPEND SCENE_MODEL_TARGETS scene_model_ros)
else()
  message(WARNING "ROS message packages not found: building scene_model without ros/ conversions")
endif()

install(TARGETS ${SCENE_MODEL_TARGETS}
  EXPORT export_scene_model
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

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
  foreach(t scene_model io voxel plane_fit export)
    ament_add_gtest(${t}_test test/core/${t}_test.cpp)
    target_link_libraries(${t}_test scene_model_core)
  endforeach()
  if(TARGET scene_model_ros)
    ament_add_gtest(ros_conversions_test test/ros/conversions_test.cpp)
    target_link_libraries(ros_conversions_test scene_model_ros)
  endif()
endif()

ament_export_targets(export_scene_model HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 yaml-cpp sensor_msgs shape_msgs geometry_msgs moveit_msgs)
ament_package()
