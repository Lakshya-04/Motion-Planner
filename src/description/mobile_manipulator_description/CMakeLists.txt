cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python profile loader: `from mobile_manipulator_description import load_profile`.
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY urdf srdf config launch rviz DESTINATION share/${PROJECT_NAME}/)
install(PROGRAMS scripts/sensor_suite_check.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(test_description test/test_description.py)
    ament_add_pytest_test(test_profile test/test_profile.py)
    ament_add_pytest_test(test_arm_swap test/test_arm_swap.py)
    ament_add_pytest_test(test_hardware_selector test/test_hardware_selector.py)
    ament_add_pytest_test(test_hardware_real test/test_hardware_real.py)
    ament_add_pytest_test(test_profile_sensors test/test_profile_sensors.py)
    ament_add_pytest_test(test_sensor_suite test/test_sensor_suite.py)
  endif()
endif()

ament_package()
