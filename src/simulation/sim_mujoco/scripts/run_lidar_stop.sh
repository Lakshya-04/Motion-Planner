#!/bin/bash
# Obstacle stop from the simulated lidar, MuJoCo. Same stack as run_stand_navigation.sh (sim_mujoco, reachability,
# stand_navigator, mobile_manipulator_description installed), plus a box standing on the floor in the straight path
# between the staging pose and the first stand. The navigator is only told about the box through /scan.
#
#   LIDAR_STOP_LIDAR=1   start the sim with lidar:=true and give the navigator obstacle_scan_topic:=/scan (default)
#   LIDAR_STOP_LIDAR=0   no lidar: the stack cannot see the box, the base is expected to drive into it
#   LIDAR_STOP_BOX="cx cy sx sy"   box centre and size on the floor, odom frame, m (default "0.18 -1.40 0.30 0.30")
#   LIDAR_STOP_STOP_M    obstacle_stop_m (default 0.30); LIDAR_STOP_HOLD_S obstacle_hold_s (default 2.0)
#   LIDAR_STOP_OUT       log directory (default /tmp/lidar_stop); STAND_MAP a prebuilt reachability map
set -u
PKG=$(ros2 pkg prefix sim_mujoco)
SHARE=$PKG/share/sim_mujoco
DESC=$(ros2 pkg prefix mobile_manipulator_description)/share/mobile_manipulator_description
PROFILE=$DESC/config/profiles/${STAND_PROFILE:-ur10e_telescopic}.yaml
OUT=${LIDAR_STOP_OUT:-/tmp/lidar_stop}
LIDAR=${LIDAR_STOP_LIDAR:-1}
BOX=${LIDAR_STOP_BOX:-0.18 -1.40 0.30 0.30}
STOP_M=${LIDAR_STOP_STOP_M:-0.30}
HOLD_S=${LIDAR_STOP_HOLD_S:-2.0}
mkdir -p "$OUT"

MAP=${STAND_MAP:-}
if [ -z "$MAP" ]; then
  MAP=$OUT/ur10e.rm4d
  RE=$(ros2 pkg prefix reachability)
  xacro "$DESC/urdf/mobile_manipulator.urdf.xacro" arm:=ur10e -o "$OUT/robot.urdf" || exit 2
  python3 "$RE/lib/reachability/urdf_to_chain.py" "$OUT/robot.urdf" --base arm_base_link --tip tcp_link \
    -o "$OUT/ur10e_chain.yaml" || exit 2
  "$RE/lib/reachability/rm4d_tool" build "$OUT/ur10e_chain.yaml" "$MAP" --samples 3000000 --resolution 0.05 \
    --extent 2.0 --zmin -1.8 --zmax 2.0 > "$OUT/map_build.log" 2>&1 || { cat "$OUT/map_build.log"; exit 2; }
fi

# the smoke wall plus a floor-standing box (0.6 m tall, so the lidar plane at 0.2 m cuts it)
python3 - "$SHARE/data/smoke_wall_scene.json" "$OUT/lidar_stop_scene.json" $BOX <<'PY' || exit 2
import json
import sys
src, dst = sys.argv[1:3]
cx, cy, sx, sy = map(float, sys.argv[3:7])
scene = json.load(open(src))
scene['obstacles'] = [{'id': 0, 'kind': 'box', 'parent_wall': 0, 'box_center_xyz': [cx, cy, 0.3],
                       'box_extents_uvn': [sx, sy, 0.6], 'box_orientation_xyzw': [0, 0, 0, 1]}]
json.dump(scene, open(dst, 'w'), indent=2)
PY

SIM_ARGS=()
NAV_ARGS=()
TEST_ARGS=()
if [ "$LIDAR" = 1 ]; then
  SIM_ARGS+=(lidar:=true)
  NAV_ARGS+=(-p obstacle_scan_topic:=/scan -p obstacle_stop_m:=$STOP_M -p obstacle_hold_s:=$HOLD_S)
  TEST_ARGS+=(--mode with)
else
  TEST_ARGS+=(--mode without)
fi

setsid ros2 launch sim_mujoco sim_mujoco.launch.py profile:=${STAND_PROFILE:-ur10e_telescopic} base_motion:=true \
  scene:=$OUT/lidar_stop_scene.json "${SIM_ARGS[@]}" "$@" > "$OUT/sim_launch.log" 2>&1 &
SIM=$!
setsid ros2 run reachability reachability_node --ros-args -r __node:=reachability_node -p map_file:=$MAP -p base_profile:=$PROFILE \
  > "$OUT/reachability.log" 2>&1 &
RN=$!
setsid ros2 run stand_navigator stand_navigator_node --ros-args -p base_profile:=$PROFILE "${NAV_ARGS[@]}" \
  > "$OUT/stand_navigator.log" 2>&1 &
SN=$!
stop_all() {
  for p in $SN $RN $SIM; do kill -INT -- "-$p" 2>/dev/null; done
  sleep 3
  for p in $SN $RN $SIM; do kill -KILL -- "-$p" 2>/dev/null; done
  wait 2>/dev/null
}
trap stop_all EXIT
timeout 300 python3 "$PKG/lib/sim_mujoco/lidar_stop_test.py" --profile "$PROFILE" "${TEST_ARGS[@]}" --box $BOX \
  --stop-m $STOP_M --sim-log "$OUT/sim_launch.log"
RC=$?
echo "lidar_stop_test exit code $RC (logs in $OUT)"
exit $RC
