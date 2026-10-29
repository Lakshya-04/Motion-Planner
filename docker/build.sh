#!/usr/bin/env bash
# Build (and optionally test) the workspace the way the Docker image expects.
#   docker/build.sh            build
#   docker/build.sh test       build, run tests, print the result summary
set -euo pipefail
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

CMAKE_ARGS=(-DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17)

# Octomap: the Ubuntu 24.04 / Jazzy apt snapshot ships MoveIt built against Ubuntu's
# octomap 1.9 while also installing ros-jazzy-octomap 1.10. If our code is linked against the
# ROS 1.10 and MoveIt's libraries pull in 1.9, two versions load into one process and the heap
# is corrupted at exit (malloc_consolidate(): invalid chunk size). Link everything against the
# one MoveIt uses.
if [ -f /usr/share/octomap/octomap-config.cmake ]; then
  CMAKE_ARGS+=(-Doctomap_DIR=/usr/share/octomap)
fi

colcon build --event-handlers console_cohesion+ --cmake-args "${CMAKE_ARGS[@]}"
if [ "${1:-}" = "test" ]; then
  colcon test --event-handlers console_direct- || true
  colcon test-result --verbose
fi
