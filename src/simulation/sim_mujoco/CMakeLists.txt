cmake_minimum_required(VERSION 3.16)
project(sim_mujoco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated-declarations)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The MuJoCo SDK (C library and headers of a mujoco-4.x release tarball) is not vendored. Point
# MUJOCO_ROOT (CMake variable or environment) at the extracted directory; without it only the
# Python tools and launch files are installed and the plugin is skipped.
set(MUJOCO_ROOT "$ENV{MUJOCO_ROOT}" CACHE PATH "Extracted mujoco-4.x SDK directory")
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h HINTS ${MUJOCO_ROOT}/include /opt/mujoco/include)
find_library(MUJOCO_LIBRARY NAMES mujoco HINTS ${MUJOCO_ROOT}/lib /opt/mujoco/lib)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/generate_mjcf scripts/smoke_test.py scripts/contact_test.py scripts/viewer.py scripts/run_smoke.sh
  scripts/run_contact.sh scripts/wall_pass_test.py scripts/run_wall_pass.sh
  scripts/record_run.py scripts/ready_pose.py scripts/run_manager_pass.sh scripts/base_motion_test.py scripts/run_base_motion.sh
  scripts/base_odom.py scripts/stand_navigation_test.py scripts/run_stand_navigation.sh
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch test/data DESTINATION share/${PROJECT_NAME})

if(MUJOCO_INCLUDE_DIR AND MUJOCO_LIBRARY)
  message(STATUS "sim_mujoco: MuJoCo found: ${MUJOCO_LIBRARY}")
  find_package(hardware_interface REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(geometry_msgs REQUIRED)

  # Ship the MuJoCo shared library next to the plugin so the install tree is self-contained.
  get_filename_component(MUJOCO_LIBRARY_REAL "${MUJOCO_LIBRARY}" REALPATH)
  get_filename_component(MUJOCO_LIBRARY_FILE "${MUJOCO_LIBRARY_REAL}" NAME)
  add_library(mujoco_imported SHARED IMPORTED)
  set_target_properties(mujoco_imported PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_LIBRARY_REAL}"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}")
  execute_process(COMMAND sh -c "readelf -d '${MUJOCO_LIBRARY_REAL}' | sed -n 's/.*SONAME.*\\[\\(.*\\)\\]/\\1/p'"
    OUTPUT_VARIABLE MUJOCO_SONAME OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(MUJOCO_SONAME)
    set_target_properties(mujoco_imported PROPERTIES IMPORTED_SONAME "${MUJOCO_SONAME}")
  endif()

  add_library(sim_mujoco_core SHARED src/mujoco_sim.cpp)
  target_include_directories(sim_mujoco_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
  target_link_libraries(sim_mujoco_core PUBLIC mujoco_imported)
  set_target_properties(sim_mujoco_core PROPERTIES INSTALL_RPATH "$ORIGIN")

  add_library(sim_mujoco_hardware SHARED src/mujoco_system_interface.cpp)
  target_link_libraries(sim_mujoco_hardware PUBLIC sim_mujoco_core)
  ament_target_dependencies(sim_mujoco_hardware PUBLIC
    hardware_interface pluginlib rclcpp rclcpp_lifecycle geometry_msgs)
  set_target_properties(sim_mujoco_hardware PROPERTIES INSTALL_RPATH "$ORIGIN")

  pluginlib_export_plugin_description_file(hardware_interface sim_mujoco.xml)

  install(TARGETS sim_mujoco_core sim_mujoco_hardware
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
  install(FILES "${MUJOCO_LIBRARY_REAL}" DESTINATION lib RENAME "${MUJOCO_SONAME}")
  install(DIRECTORY include/ DESTINATION include)
  install(FILES sim_mujoco.xml DESTINATION share/${PROJECT_NAME})

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_mujoco_sim test/test_mujoco_sim.cpp)
    target_link_libraries(test_mujoco_sim sim_mujoco_core)
    target_compile_definitions(test_mujoco_sim PRIVATE
      SIM_MUJOCO_TEST_DATA="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  endif()
else()
  message(WARNING "sim_mujoco: MuJoCo SDK not found (set MUJOCO_ROOT); the plugin is not built.")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(sim_mujoco_python test/ TIMEOUT 300)
endif()

ament_package()
