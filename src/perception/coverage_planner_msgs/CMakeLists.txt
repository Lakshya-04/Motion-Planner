cmake_minimum_required(VERSION 3.14.4)
project(coverage_planner_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WallPlane.msg"
  "msg/ObstacleRegion.msg"
  "msg/MotionPlanSegment.msg"
  "msg/CoveragePlan.msg"
  "srv/GenerateCoveragePath.srv"
  DEPENDENCIES nav_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime nav_msgs geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    set(ament_cmake_uncrustify_FOUND TRUE)
    set(ament_cmake_flake8_FOUND TRUE)
    set(ament_cmake_pep257_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

ament_package()
