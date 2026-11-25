cmake_minimum_required(VERSION 3.8)
project(standoff_advisor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(reachability REQUIRED)

# core: ROS-free. Standoff sweep over an RM4D map.
add_library(standoff_advisor_core src/standoff_sweep.cpp)
target_include_directories(standoff_advisor_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(standoff_advisor_core PUBLIC reachability::reachability_core Eigen3::Eigen)
target_compile_features(standoff_advisor_core PUBLIC cxx_std_17)

add_executable(standoff_sweep apps/standoff_sweep.cpp)
target_link_libraries(standoff_sweep standoff_advisor_core yaml-cpp)

install(TARGETS standoff_advisor_core
  EXPORT export_standoff_advisor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS standoff_sweep DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/standoff_plot.py scripts/profile_to_map.sh DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(standoff_sweep_test test/standoff_sweep_test.cpp)
  target_link_libraries(standoff_sweep_test standoff_advisor_core)
endif()

ament_export_targets(export_standoff_advisor HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 reachability)
ament_package()
