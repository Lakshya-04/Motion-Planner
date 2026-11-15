cmake_minimum_required(VERSION 3.14.4)
project(sim_isaac)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(sim_isaac_stub test/test_stub.py)
endif()

ament_package()
