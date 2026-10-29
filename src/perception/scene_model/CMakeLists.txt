cmake_minimum_required(VERSION 3.8)
project(scene_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

install(TARGETS scene_model_core
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
    ament_lint_auto_find_test_dependencies()
  endif()
  foreach(t scene_model io voxel plane_fit export)
    ament_add_gtest(${t}_test test/core/${t}_test.cpp)
    target_link_libraries(${t}_test scene_model_core)
  endforeach()
endif()

ament_export_targets(export_scene_model HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 yaml-cpp)
ament_package()
