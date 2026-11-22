cmake_minimum_required(VERSION 3.8)
project(lift_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ---- core: plain C++17, no ROS ------------------------------------------------------------------
# Linked into the plugin's shared library, so it must be position independent.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(lift_hardware_core STATIC
  src/core/fault.cpp
  src/core/profile.cpp
  src/core/lift_config.cpp
  src/core/lift_supervisor.cpp
  src/core/mock_transport.cpp
  src/core/scripted_transport.cpp
  src/core/device_transport.cpp
  src/core/transport_factory.cpp
)
target_include_directories(lift_hardware_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(lift_hardware_core PUBLIC cxx_std_17)

# ---- ros: the ros2_control plugin ---------------------------------------------------------------
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(lift_hardware_ros SHARED src/ros/prismatic_lift_system.cpp)
target_link_libraries(lift_hardware_ros PUBLIC lift_hardware_core)
ament_target_dependencies(lift_hardware_ros PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface lift_hardware.xml)

install(TARGETS lift_hardware_core lift_hardware_ros
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES lift_hardware.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  foreach(name profile lift_config mock_transport supervisor)
    ament_add_gtest(test_${name} test/core/test_${name}.cpp)
    target_link_libraries(test_${name} lift_hardware_core)
  endforeach()
endif()

ament_package()
