cmake_minimum_required(VERSION 3.8)
project(cartesian_tracking_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(diff_ik_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(kinematics_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)

add_library(cartesian_tracking_controller SHARED
  src/cartesian_tracking_controller.cpp
)
target_include_directories(cartesian_tracking_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(cartesian_tracking_controller PUBLIC diff_ik_core::diff_ik_core)
ament_target_dependencies(cartesian_tracking_controller PUBLIC
  controller_interface
  Eigen3
  geometry_msgs
  hardware_interface
  kinematics_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  urdf
)
pluginlib_export_plugin_description_file(controller_interface cartesian_tracking_controller.xml)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES cartesian_tracking_controller.xml DESTINATION share/${PROJECT_NAME})
install(TARGETS cartesian_tracking_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(cartesian_tracking_controller)
ament_export_dependencies(controller_interface diff_ik_core hardware_interface pluginlib rclcpp_lifecycle realtime_tools)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # The test robot is the demo URDF of compliant_trajectory_controller, read from the source tree
  # so this package does not have to build the MoveIt-heavy one.
  ament_add_gtest(test_cartesian_tracking_controller
    test/test_cartesian_tracking_controller.cpp
    TIMEOUT 120
  )
  if(TARGET test_cartesian_tracking_controller)
    target_link_libraries(test_cartesian_tracking_controller cartesian_tracking_controller)
    target_compile_definitions(test_cartesian_tracking_controller PRIVATE
      DEMO_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../compliant_trajectory_controller/urdf/jtac_demo_robot.urdf")
  endif()

  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    # Same skips as the neighbouring controller packages (no copyright headers, own style).
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
