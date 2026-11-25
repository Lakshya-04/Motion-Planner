cmake_minimum_required(VERSION 3.8)
project(sim_overlay)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/gz_overlay.py
  scripts/overlay_dump.py
  scripts/coverage_paint.py
  scripts/coverage_replay.py
  scripts/touch_up_plan.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_overlay test/test_overlay.py)
  ament_add_pytest_test(test_gz_commands test/test_gz_commands.py)
  ament_add_pytest_test(test_coverage_paint test/test_coverage_paint.py)
  ament_add_pytest_test(test_touch_up test/test_touch_up.py)
  ament_add_pytest_test(test_mujoco_overlay test/test_mujoco_overlay.py)
endif()

ament_package()
