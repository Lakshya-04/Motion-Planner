cmake_minimum_required(VERSION 3.8)
project(sim_overlay)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/gz_overlay.py
  scripts/overlay_dump.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY patches DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_overlay test/test_overlay.py)
  ament_add_pytest_test(test_gz_commands test/test_gz_commands.py)
  ament_add_pytest_test(test_mujoco_overlay test/test_mujoco_overlay.py)
endif()

ament_package()
