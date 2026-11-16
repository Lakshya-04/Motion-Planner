#!/bin/bash
# Headless wall docking check in a sourced ROS environment with sim_mujoco, wall_detector, stand_navigator and
# mobile_manipulator_description installed: launches the sim with base_motion:=true, depth_sensor:=true and the smoke
# wall, starts wall_detector with target_frame odom (a fixed world plane, so the base moving does not smear it) and
# wall_docking_node, then scripts/wall_docking_test.py drives the base to a start pose off the target standoff and
# yawed, sends WallDock, and reports the final standoff and yaw error from /odom.
#
#   DOCK_PROFILE=ur10e_telescopic   robot profile (drive type comes from its base: section; try a diff profile,
#                                   the node then never sends a lateral velocity to the omni sim base)
#   DOCK_STANDOFF=0.55  DOCK_OFFSET=0.4  DOCK_YAW_DEG=15   the target and the start offsets
# Extra arguments go to the sim launch.
set -u
PKG=$(ros2 pkg prefix sim_mujoco)
SHARE=$PKG/share/sim_mujoco
DESC=$(ros2 pkg prefix mobile_manipulator_description)/share/mobile_manipulator_description
PROFILE=$DESC/config/profiles/${DOCK_PROFILE:-ur10e_telescopic}.yaml
OUT=${WALL_DOCKING_OUT:-/tmp/wall_docking}
mkdir -p "$OUT"
rm -f "$OUT/wall_scene.json" "$OUT/walls.pcd"

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

start sim ros2 launch sim_mujoco sim_mujoco.launch.py profile:=${DOCK_PROFILE:-ur10e_telescopic} base_motion:=true \
  depth_sensor:=true depth_sensor_pose:="0.1 0 1.15 0 0 0" depth_sensor_noise_m:=0.003 \
  scene:=$SHARE/data/smoke_wall_scene.json "$@"
start detector ros2 launch wall_detector wall_detector.launch.py output_dir:=$OUT target_frame:=odom \
  max_range_m:=4.0 accumulate_frames:=3
# the reference point is the footprint origin: the standoff is the base_footprint to wall distance
start docking ros2 run stand_navigator wall_docking_node --ros-args -p base_profile:=$PROFILE -p ref_xy:="[0.0, 0.0]"
timeout 300 python3 "$PKG/lib/sim_mujoco/wall_docking_test.py" --profile "$PROFILE" \
  --standoff-m ${DOCK_STANDOFF:-0.55} --offset-m ${DOCK_OFFSET:-0.4} --yaw-deg ${DOCK_YAW_DEG:-15}
RC=$?
echo "wall_docking_test exit code $RC (logs: $OUT)"
exit $RC
