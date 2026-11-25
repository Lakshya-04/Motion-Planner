#!/usr/bin/env bash
# Build (and optionally test) the workspace the way the Docker image expects.
#   docker/build.sh            build
#   docker/build.sh test       build, run tests, print the result summary
set -eo pipefail
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

CMAKE_ARGS=(-DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17)

# Compiler cache + mold (image default, docs/docker.md "Compiler cache (default)"). --cmake-args on the
# command line replaces the colcon defaults file, so repeat them here; harmless when ccache or mold is absent.
# Mount a host dir at /ccache to keep the cache between containers:  -v $HOME/.cache/mp-ccache:/ccache
export CCACHE_DIR="${CCACHE_DIR:-/ccache}"
if command -v ccache >/dev/null 2>&1 && [ -w "$CCACHE_DIR" ]; then
  CMAKE_ARGS+=(-DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache)
fi
if command -v mold >/dev/null 2>&1; then
  CMAKE_ARGS+=(-DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=mold -DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=mold
               -DCMAKE_MODULE_LINKER_FLAGS=-fuse-ld=mold)
fi


# Octomap: the Ubuntu 24.04 / Jazzy apt snapshot ships MoveIt built against Ubuntu's
# octomap 1.9 while also installing ros-jazzy-octomap 1.10. If our code is linked against the
# ROS 1.10 and MoveIt's libraries pull in 1.9, two versions load into one process and the heap
# is corrupted at exit (malloc_consolidate(): invalid chunk size). Link everything against the
# one MoveIt uses.
if [ -f /usr/share/octomap/octomap-config.cmake ]; then
  CMAKE_ARGS+=(-Doctomap_DIR=/usr/share/octomap)
fi

[ -z "${CCACHE_STATS:-}" ] || ccache -z >/dev/null 2>&1 || true
colcon build --event-handlers console_cohesion+ --cmake-args "${CMAKE_ARGS[@]}"
if [ "${1:-}" = "test" ]; then
  colcon test --event-handlers console_direct- || true
  colcon test-result --verbose
fi
[ -z "${CCACHE_STATS:-}" ] || ccache -s || true
