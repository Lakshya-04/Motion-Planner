cmake_minimum_required(VERSION 3.8)
project(manipulation_bt_prototype)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Opt-in: the rest of the workspace builds without BehaviorTree.CPP. When it is absent this package is empty.
find_package(behaviortree_cpp QUIET)
find_package(nlohmann_json QUIET)

if(behaviortree_cpp_FOUND AND nlohmann_json_FOUND)
  message(STATUS "manipulation_bt_prototype: behaviortree_cpp ${behaviortree_cpp_VERSION} found, building")
  # mission_fsm.hpp and mission_ledger.hpp are header-only and ROS-free; they are used unchanged from the manager's
  # source tree (a prototype shortcut: no link to the manager library, which would pull MoveIt and PCL in).
  set(MM_CORE_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../manipulation_manager/include)

  add_library(${PROJECT_NAME} INTERFACE)
  target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${MM_CORE_INCLUDE}>)
  target_link_libraries(${PROJECT_NAME} INTERFACE BT::behaviortree_cpp nlohmann_json::nlohmann_json)

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(bt_prototype_test test/bt_prototype_test.cpp)
    if(TARGET bt_prototype_test)
      target_link_libraries(bt_prototype_test ${PROJECT_NAME})
      target_compile_definitions(bt_prototype_test PRIVATE
        BT_PROTOTYPE_TREE_FILE="${CMAKE_CURRENT_SOURCE_DIR}/trees/prototype_mission.xml"
        BT_TARGET_TREE_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/behavior_trees/coverage_mission.xml")
    endif()
  endif()
else()
  message(STATUS "manipulation_bt_prototype: behaviortree_cpp not found, skipped (opt-in package)")
endif()

ament_package()
