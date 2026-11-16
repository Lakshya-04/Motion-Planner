#!/bin/bash
# Live octomap check in a sourced ROS environment with sim_mujoco, wall_detector and moveit_ros_perception
# installed: launches the sim with depth_sensor:=true, move_group:=true live_octomap:=true and the smoke wall (x = 0.98 m
# in base_footprint). MoveIt's PointCloudOctomapUpdater builds the octomap from /depth_sensor/points; nothing is loaded
# from a file. octomap_esdf_node makes a distance field from the octomap. scripts/live_octomap_test.py checks the
# planning scene's octomap, the ESDF and a collision-checked Cartesian approach to the wall, then everything stops.
#
#   LIVE_OCTOMAP_OUT (default /tmp/live_octomap)   logs     LIVE_OCTOMAP_NOISE  sensor range noise, m (default 0.003)
# Extra arguments go to the sim launch.
set -u
PKG=$(ros2 pkg prefix sim_mujoco)
SHARE=$PKG/share/sim_mujoco
OUT=${LIVE_OCTOMAP_OUT:-/tmp/live_octomap}
mkdir -p "$OUT"
PIDS=()
start() {  # name, command...
  local name=$1; shift
  setsid "$@" > "$OUT/$name.log" 2>&1 &
  PIDS+=($!)
}
stop_all() {
  for pid in "${PIDS[@]}"; do kill -INT -- "-$pid" 2>/dev/null; done
  for _ in $(seq 100); do
    alive=0
    for pid in "${PIDS[@]}"; do kill -0 -- "-$pid" 2>/dev/null && alive=1; done
    [ $alive = 0 ] && break
    sleep 0.2
  done
  for pid in "${PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null; wait "$pid" 2>/dev/null; done
}
trap stop_all EXIT

start sim ros2 launch sim_mujoco sim_mujoco.launch.py profile:=ur10e_telescopic move_group:=true \
  live_octomap:=true depth_sensor:=true depth_sensor_pose:="0.1 0 1.15 0 0 0" \
  depth_sensor_noise_m:=${LIVE_OCTOMAP_NOISE:-0.003} scene:=$SHARE/data/smoke_wall_scene.json "$@"
start esdf ros2 run wall_detector octomap_esdf_node --ros-args -r __node:=octomap_esdf \
  -p center:="[0.8, 0.0, 1.0]" -p dims:="[1.6, 1.6, 1.6]" -p voxel_size:=0.05
timeout 400 python3 "$PKG/lib/sim_mujoco/live_octomap_test.py" --esdf
RC=$?
echo "live_octomap_test exit code $RC (logs: $OUT)"
exit $RC
