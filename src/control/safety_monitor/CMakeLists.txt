cmake_minimum_required(VERSION 3.8)
project(safety_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# May be linked into a shared library (a controller plugin), so the static library must be PIC.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME}
  src/sphere_model.cpp
  src/curobo_yaml.cpp
  src/distance_source.cpp
  src/safety_monitor.cpp
  src/safety_chain.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen yaml-cpp)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(${PROJECT_NAME}Targets)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(curobo_yaml_test test/curobo_yaml_test.cpp)
  target_link_libraries(curobo_yaml_test ${PROJECT_NAME})

  ament_add_gtest(sphere_model_test test/sphere_model_test.cpp)
  target_link_libraries(sphere_model_test ${PROJECT_NAME})

  ament_add_gtest(distance_source_test test/distance_source_test.cpp)
  target_link_libraries(distance_source_test ${PROJECT_NAME})

  ament_add_gtest(safety_monitor_test test/safety_monitor_test.cpp)
  target_link_libraries(safety_monitor_test ${PROJECT_NAME})

  ament_add_gtest(safety_chain_test test/safety_chain_test.cpp)
  target_link_libraries(safety_chain_test ${PROJECT_NAME})

  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    # No per-file copyright headers in this repo yet -- same skips as the neighbouring packages.
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    set(ament_cmake_uncrustify_FOUND TRUE)
    set(ament_cmake_flake8_FOUND TRUE)
    set(ament_cmake_pep257_FOUND TRUE)
    set(ament_cmake_xmllint_FOUND TRUE)
    set(ament_cmake_lint_cmake_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

ament_package()
