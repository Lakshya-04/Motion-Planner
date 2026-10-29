# 0004: C++17 baseline, C++20 allowed in ROS-free core later

- Status: accepted
- Date: 2026-10-30

## Context

C++20 and C++23 exist. The ROS 2 ecosystem we link against (rclcpp, MoveIt, Nav2,
ros2_control, pluginlib) is built as C++17, and the supported compilers on our target
distros (GCC 11 on Humble / Ubuntu 22.04, newer on Jazzy / Ubuntu 24.04) cover C++17
fully and C++20 mostly. VAMP, Eigen 3.4 and PCL build as C++17.

## Decision

- All packages build as `cxx_std_17` (`CMAKE_CXX_STANDARD 17`, extensions off).
- A ROS-free `core/` library may move to C++20 later, one package at a time, only if it has
  a concrete need (concepts, `std::span`, ranges, designated initializers) and CI-free local
  builds pass on the oldest supported compiler. `ros/` shells stay C++17 so they match
  the ROS headers they include.
- Do not use C++23 features.

## Consequences

Maximum compatibility with ROS and third-party headers now; a clear, small path to C++20
in `core/` when it pays off.
