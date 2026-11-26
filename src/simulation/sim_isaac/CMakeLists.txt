cmake_minimum_required(VERSION 3.14.4)
project(sim_isaac)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
# Isaac Sim runs in its own container; these are plain scripts, nothing here links against it
install(PROGRAMS scripts/run_isaac.py scripts/urdf_to_usd.py scripts/smoke_test.py scripts/make_resolved_urdf.py
  scripts/run_smoke.sh scripts/isaac_common.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(sim_isaac_stub test/test_stub.py)
  ament_add_pytest_test(sim_isaac_common test/test_isaac_common.py)
endif()

ament_package()
