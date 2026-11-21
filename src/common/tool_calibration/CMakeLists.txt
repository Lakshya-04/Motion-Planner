cmake_minimum_required(VERSION 3.8)
project(tool_calibration)

find_package(ament_cmake REQUIRED)
find_package(core_types REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
target_link_libraries(${PROJECT_NAME} INTERFACE core_types::core_types nlohmann_json::nlohmann_json)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
)

ament_export_targets(${PROJECT_NAME}Targets)
ament_export_dependencies(core_types nlohmann_json)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tool_calibration test/test_calibration_record.cpp)
  target_link_libraries(test_tool_calibration ${PROJECT_NAME})

  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    # Same lint skips as core_types.
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
