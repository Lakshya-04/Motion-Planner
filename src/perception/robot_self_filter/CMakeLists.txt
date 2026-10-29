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

install(TARGETS robot_self_filter_core
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
endif()

ament_export_targets(export_robot_self_filter HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 scene_model)
ament_package()
