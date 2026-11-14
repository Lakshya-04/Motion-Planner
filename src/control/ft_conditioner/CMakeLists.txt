cmake_minimum_required(VERSION 3.14.4)
project(ft_conditioner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(core_types REQUIRED)
find_package(force_core REQUIRED)

add_library(ft_conditioner_lib src/wrench_conditioner_node.cpp)
target_include_directories(ft_conditioner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ft_conditioner_lib PUBLIC cxx_std_17)
target_link_libraries(ft_conditioner_lib PUBLIC force_core::force_core core_types::core_types)
ament_target_dependencies(ft_conditioner_lib PUBLIC rclcpp geometry_msgs std_srvs tf2_ros)

add_executable(wrench_conditioner_node src/main.cpp)
target_link_libraries(wrench_conditioner_node ft_conditioner_lib)

install(TARGETS ft_conditioner_lib wrench_conditioner_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(wrench_conditioner_node_test test/wrench_conditioner_node_test.cpp)
  target_link_libraries(wrench_conditioner_node_test ft_conditioner_lib)
endif()

ament_package()
