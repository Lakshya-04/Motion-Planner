cmake_minimum_required(VERSION 3.8)
project(teleop_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME}
  src/teleop_mode_manager.cpp
  src/jog_shaper.cpp
  src/jog_limiter.cpp
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

  ament_add_gtest(teleop_mode_manager_test test/teleop_mode_manager_test.cpp)
  target_link_libraries(teleop_mode_manager_test ${PROJECT_NAME})

  ament_add_gtest(jog_shaper_test test/jog_shaper_test.cpp)
  target_link_libraries(jog_shaper_test ${PROJECT_NAME})

  ament_add_gtest(jog_limiter_test test/jog_limiter_test.cpp)
  target_link_libraries(jog_limiter_test ${PROJECT_NAME})

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
endif()

ament_package()
