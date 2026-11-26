cmake_minimum_required(VERSION 3.8)
project(sim_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY urdf config launch worlds DESTINATION share/${PROJECT_NAME}/)
install(PROGRAMS scripts/wall_world_gen.py scripts/smoke_test.py scripts/gravity_check.py scripts/contact_control_test.py scripts/contact_ground_truth.py scripts/record_run.py scripts/ready_pose.py scripts/wait_arm_settled.py scripts/run_manager_pass.sh scripts/base_motion_test.py scripts/depth_wall_test.py scripts/run_base_motion.sh scripts/run_depth_wall.sh scripts/run_lidar_stop.sh scripts/lidar_stop_test.py scripts/run_spray_pass.sh scripts/run_multi_stand_pass.sh scripts/gz_pass_common.sh scripts/capture_still.py scripts/shift_wall_scene.py scripts/execute_coverage_client.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY test/data DESTINATION share/${PROJECT_NAME}/test)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(test_wall_world test/test_wall_world.py)
    ament_add_pytest_test(test_description_gazebo test/test_description_gazebo.py)
    ament_add_pytest_test(test_contact_script test/test_contact_script.py)
    ament_add_pytest_test(test_contact_summary test/test_contact_summary.py)
  endif()
endif()

ament_package()
