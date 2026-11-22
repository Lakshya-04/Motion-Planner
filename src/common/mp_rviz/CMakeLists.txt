cmake_minimum_required(VERSION 3.8)
project(mp_rviz)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch profiles DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/mp_viz_markers.py scripts/mp_viz_scene.py scripts/make_full_wall_fixture.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/mp_viz_logic.py scripts/mp_viz_scene_logic.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_profiles test/test_profiles.py)
  ament_add_pytest_test(test_viz_logic test/test_viz_logic.py)
  ament_add_pytest_test(test_viz_scene_logic test/test_viz_scene_logic.py)
endif()

ament_package()
