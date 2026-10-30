cmake_minimum_required(VERSION 3.8)
project(scene_adapters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(scene_model REQUIRED)
find_package(motion_planner REQUIRED)
find_package(core_types REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(scene_adapters src/world_geometry.cpp)
target_include_directories(scene_adapters PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(scene_adapters PUBLIC
  scene_model::scene_model_core motion_planner::motion_planner_core core_types::core_types Eigen3::Eigen)

install(TARGETS scene_adapters EXPORT export_scene_adapters
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(world_geometry_test test/world_geometry_test.cpp)
  target_link_libraries(world_geometry_test scene_adapters)
endif()

ament_export_targets(export_scene_adapters HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(scene_model motion_planner core_types Eigen3)
ament_package()
