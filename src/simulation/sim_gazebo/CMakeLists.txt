cmake_minimum_required(VERSION 3.8)
project(sim_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY urdf config launch worlds DESTINATION share/${PROJECT_NAME}/)
install(PROGRAMS scripts/wall_world_gen.py scripts/smoke_test.py scripts/contact_ground_truth.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY test/data DESTINATION share/${PROJECT_NAME}/test)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(test_wall_world test/test_wall_world.py)
    ament_add_pytest_test(test_description_gazebo test/test_description_gazebo.py)
  endif()
endif()

ament_package()
