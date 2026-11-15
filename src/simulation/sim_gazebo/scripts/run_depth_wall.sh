#!/bin/bash
# Headless live-perception check in a sourced ROS environment with sim_gazebo, wall_detector and coverage_planner
# installed: launches the sim with depth_sensor:=true and the smoke wall world (front face x = 0.915 m in base_footprint, normal -x),
# starts wall_detector (PointCloud2 -> wall_scene.json + walls.pcd) and the coverage planner nodes, runs
# scripts/depth_wall_test.py (fitted plane vs the known wall, coverage planner accepts the detected scene), stops all.
#
#   DEPTH_WALL_MOVEIT=1  also start move_group and check the detected wall appears in its planning scene
#   DEPTH_WALL_NOISE=0.003  range noise of the sensor, m (default 0.003)
# Extra arguments go to the sim launch (e.g. base_motion:=true).
set -u
PKG=$(ros2 pkg prefix sim_gazebo)
SHARE=$PKG/share/sim_gazebo
OUT=${DEPTH_WALL_OUT:-/tmp/depth_wall}
mkdir -p "$OUT"
rm -f "$OUT/wall_scene.json" "$OUT/walls.pcd"
MOVEIT=${DEPTH_WALL_MOVEIT:-0}
SIM_EXTRA=()
DET_EXTRA=()
TEST_EXTRA=()
if [ "$MOVEIT" = 1 ]; then
  SIM_EXTRA+=(move_group:=true)
  DET_EXTRA+=(publish_collision_object:=true)
  TEST_EXTRA+=(--moveit)
fi

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

start sim ros2 launch sim_gazebo sim.launch.py profile:=ur10e_telescopic depth_sensor:=true \
  depth_sensor_pose:="0.1 0 1.15 0 0 0" depth_sensor_noise_m:=${DEPTH_WALL_NOISE:-0.003} \
  wall_scene:=$SHARE/test/data/smoke_wall_scene.json standoff:=0.915 "${SIM_EXTRA[@]}" "$@"
start detector ros2 launch wall_detector wall_detector.launch.py output_dir:=$OUT target_frame:=base_footprint \
  max_range_m:=4.0 "${DET_EXTRA[@]}"
start coverage ros2 launch coverage_planner wall_coverage_service.launch.py wall_scene_json:=$OUT/wall_scene.json \
  resolution:=0.05
timeout 400 python3 "$PKG/lib/sim_gazebo/depth_wall_test.py" "${TEST_EXTRA[@]}"
RC=$?
echo "depth_wall_test exit code $RC (logs: $OUT)"
exit $RC
