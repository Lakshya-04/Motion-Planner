#!/bin/bash
# Headless check of robot_self_filter on the Gazebo depth cloud (which contains the robot's own links), in a sourced ROS
# environment with sim_gazebo, robot_self_filter and wall_detector installed: launches the sim with depth_sensor:=true on the
# smoke wall, starts the self filter (/depth_sensor/points -> /depth_sensor/points_filtered) and one wall_detector on each
# cloud, then runs scripts/self_filter_check.py with the arm at its start pose and in each pose of SELF_FILTER_POSES.
#
#   SELF_FILTER_POSES="lift,j1..j6;lift,j1..j6"  SELF_FILTER_OUT=/tmp/self_filter_check  SELF_FILTER_SENSOR_POSE="x y z r p y"  SELF_FILTER_EXTRA="depth_realism:=typical" (extra sim launch arguments, no spaces)
set -u
PKG=$(ros2 pkg prefix sim_gazebo)
SHARE=$PKG/share/sim_gazebo
OUT=${SELF_FILTER_OUT:-/tmp/self_filter_check}
mkdir -p "$OUT/raw" "$OUT/filtered"
rm -f "$OUT"/raw/wall_scene.json "$OUT"/filtered/wall_scene.json

PIDS=()
start() { local name=$1; shift; setsid "$@" > "$OUT/$name.log" 2>&1 & PIDS+=($!); }
stop_all() {
  for pid in "${PIDS[@]}"; do kill -INT -- "-$pid" 2>/dev/null; done
  sleep 3
  for pid in "${PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null; wait "$pid" 2>/dev/null; done
  pkill -KILL -f "gz sim" 2>/dev/null
}
trap stop_all EXIT

# shellcheck disable=SC2086
start sim ros2 launch sim_gazebo sim.launch.py profile:=ur10e_telescopic controller:=jtac jtac_position_only:=true \
  ft_conditioner:=true payload_mass_kg:=0.8 depth_sensor:=true depth_sensor_pose:="${SELF_FILTER_SENSOR_POSE:-0.1 0 1.15 0 0 0}" \
  wall_scene:=$SHARE/test/data/smoke_wall_scene.json standoff:=0.915 ${SELF_FILTER_EXTRA:-}
# tool_tip_link (the tool's collision) sits on a joint nobody publishes in TF, so it is left out of the filter's links
start self_filter ros2 run robot_self_filter self_filter_node --ros-args -r __node:=self_filter \
  --params-file $(ros2 pkg prefix robot_self_filter)/share/robot_self_filter/config/self_filter.yaml -p links:="[base_link, lift_base_link, lift_carriage_link, lift_stage_1_link, lift_stage_2_link, arm_base_link_inertia, arm_shoulder_link, arm_upper_arm_link, arm_forearm_link, arm_wrist_1_link, arm_wrist_2_link, arm_wrist_3_link]" \
  -r cloud_in:=/depth_sensor/points -r cloud_out:=/depth_sensor/points_filtered
for which in raw filtered; do
  topic=/depth_sensor/points; [ $which = filtered ] && topic=/depth_sensor/points_filtered
  start det_$which ros2 run wall_detector wall_detector_node --ros-args -r __node:=wd_$which -p cloud_topic:=$topic \
    -p target_frame:=base_footprint -p output_dir:=$OUT/$which -p max_range_m:=4.0 -p accumulate_frames:=5
done
sleep 45   # gz, controllers, detectors up

CHECK="python3 $PKG/lib/sim_gazebo/self_filter_check.py --scene-raw $OUT/raw/wall_scene.json --scene-filtered $OUT/filtered/wall_scene.json"
RC=0
$CHECK --label "start pose" || RC=1
# arm poses (lift, then the six UR10e joints); the sensor is 1.15 m up and looks along +x, so the poses put links in its view
POSES=${SELF_FILTER_POSES:-"0.4,-0.0363,0.2528,1.1964,1.6924,-1.5345,1.5708;0.4,0,-0.6,1.2,-0.6,0,0;0.0,0,-0.3,0.6,-0.3,0,0;0.2,0.5,-0.5,1.0,-0.5,0,0"}
IFS=';' read -ra POSE_LIST <<< "$POSES"
for pose in "${POSE_LIST[@]}"; do
  python3 "$PKG/lib/sim_gazebo/ready_pose.py" --joints "$pose" --seconds 6 > "$OUT/ready_pose.log" 2>&1 || echo "ready_pose failed (see $OUT/ready_pose.log)"
  sleep 4
  rm -f "$OUT"/raw/wall_scene.json "$OUT"/filtered/wall_scene.json
  for _ in $(seq 60); do [ -s "$OUT/raw/wall_scene.json" ] && [ -s "$OUT/filtered/wall_scene.json" ] && break; sleep 1; done
  $CHECK --label "pose $pose" || RC=1
done
echo "self_filter_check exit code $RC (logs: $OUT)"
exit $RC
