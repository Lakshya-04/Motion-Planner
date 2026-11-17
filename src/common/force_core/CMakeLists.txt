cmake_minimum_required(VERSION 3.8)
project(force_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(core_types REQUIRED)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME}
  src/force_stage_fsm.cpp
  src/force_stage_sequencer.cpp
  src/force_limiter.cpp
  src/wrench_conditioning.cpp
  src/payload_excitation.cpp
  src/bias_tracker.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# Threads::Threads -- force_stage_sequencer.cpp's own trim loop (ForceStageSequencer::trimLoop())
# runs on a real std::thread, see that file's own "FORCE TRIM" doc comment.
target_link_libraries(${PROJECT_NAME} PUBLIC core_types::core_types Threads::Threads)

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
ament_export_dependencies(core_types Threads)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Pure logic, no clock, no ROS -- see force_stage_fsm.hpp's own portability constraint
  # (only cstdint/string/vector/array + core_types headers), which is what made this
  # extraction from manipulation_manager a plain `git mv`, not a rewrite.
  ament_add_gtest(force_stage_fsm_test test/force_stage_fsm_test.cpp)
  target_link_libraries(force_stage_fsm_test ${PROJECT_NAME})

  ament_add_gtest(force_stage_sequencer_test test/force_stage_sequencer_test.cpp)
  target_link_libraries(force_stage_sequencer_test ${PROJECT_NAME})

  ament_add_gtest(force_limiter_test test/force_limiter_test.cpp)
  target_link_libraries(force_limiter_test ${PROJECT_NAME})

  ament_add_gtest(wrench_conditioning_test test/wrench_conditioning_test.cpp)
  target_link_libraries(wrench_conditioning_test ${PROJECT_NAME})

  ament_add_gtest(payload_excitation_test test/payload_excitation_test.cpp)
  target_link_libraries(payload_excitation_test ${PROJECT_NAME})

  ament_add_gtest(bias_tracker_test test/bias_tracker_test.cpp)
  target_link_libraries(bias_tracker_test ${PROJECT_NAME})

  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    # No per-file copyright headers in this repo yet -- matches core_types'/motion_planner's
    # own skip.
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
