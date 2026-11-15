cmake_minimum_required(VERSION 3.8)
project(ft_bridge_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(ft_bridge_hardware SHARED
  src/ft_bridge_hardware_interface.cpp
)

target_include_directories(ft_bridge_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ft_bridge_hardware
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
  geometry_msgs
)

pluginlib_export_plugin_description_file(hardware_interface ft_bridge_hardware.xml)

# The same bridge under gz_ros2_control's plugin base class (Gazebo only accepts that one), built
# only where gz_ros2_control is installed (the sim-gazebo image), so the other images are unaffected.
find_package(gz-sim8 QUIET)  # gz_ros2_control's exported targets need it found first
if(gz-sim8_FOUND)
  find_package(gz_ros2_control QUIET)
endif()
if(gz-sim8_FOUND AND gz_ros2_control_FOUND)
  add_library(ft_bridge_hardware_gz SHARED src/ft_bridge_sim_system.cpp)
  target_link_libraries(ft_bridge_hardware_gz ft_bridge_hardware)
  ament_target_dependencies(ft_bridge_hardware_gz gz_ros2_control)
  pluginlib_export_plugin_description_file(gz_ros2_control ft_bridge_sim_system.xml)
  install(TARGETS ft_bridge_hardware_gz ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
  install(FILES ft_bridge_sim_system.xml DESTINATION share/${PROJECT_NAME})
endif()

install(TARGETS
  ft_bridge_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES ft_bridge_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_wrench_bias test/test_wrench_bias.cpp)
  target_include_directories(test_wrench_bias PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_ft_bridge_config test/test_ft_bridge_config.cpp)
  target_include_directories(test_ft_bridge_config PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

ament_package()
