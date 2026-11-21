cmake_minimum_required(VERSION 3.8)
project(diff_ik_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# A later controller plugin (a shared library) will link this static library, so it must be PIC.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME}
  src/jacobian_quality.cpp
  src/null_space_costs.cpp
  src/diff_ik.cpp
  src/pose_error.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

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
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(jacobian_quality_test test/jacobian_quality_test.cpp)
  target_link_libraries(jacobian_quality_test ${PROJECT_NAME})

  ament_add_gtest(diff_ik_test test/diff_ik_test.cpp)
  target_link_libraries(diff_ik_test ${PROJECT_NAME})

  ament_add_gtest(pose_error_test test/pose_error_test.cpp)
  target_link_libraries(pose_error_test ${PROJECT_NAME})

  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    # No per-file copyright headers in this repo yet -- same skip as the neighbouring cores.
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
