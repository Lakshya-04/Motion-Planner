#!/bin/bash
# Headless stand navigation test in a sourced ROS environment with sim_mujoco, reachability,
# stand_navigator and mobile_manipulator_description installed: builds a UR10e reachability map (unless
# STAND_MAP points at one), launches the sim with base_motion:=true and the smoke wall, starts
# reachability_node (with the base profile) and stand_navigator_node, runs scripts/stand_navigation_test.py
# (plan_stands for the wall poses -> drive the arm base to the first stand -> check arrival, arm, F/T).
set -u
PKG=$(ros2 pkg prefix sim_mujoco)
SHARE=$PKG/share/sim_mujoco
DESC=$(ros2 pkg prefix mobile_manipulator_description)/share/mobile_manipulator_description
PROFILE=$DESC/config/profiles/${STAND_PROFILE:-ur10e_telescopic}.yaml
OUT=${STAND_NAV_OUT:-/tmp/stand_navigation}
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

setsid ros2 launch sim_mujoco sim_mujoco.launch.py profile:=${STAND_PROFILE:-ur10e_telescopic} base_motion:=true \
  scene:=$SHARE/data/smoke_wall_scene.json "$@" > "$OUT/sim_launch.log" 2>&1 &
SIM=$!
setsid ros2 run reachability reachability_node --ros-args -r __node:=reachability_node -p map_file:=$MAP -p base_profile:=$PROFILE \
  > "$OUT/reachability.log" 2>&1 &
RN=$!
setsid ros2 run stand_navigator stand_navigator_node --ros-args -p base_profile:=$PROFILE \
  > "$OUT/stand_navigator.log" 2>&1 &
SN=$!
stop_all() {
  for p in $SN $RN $SIM; do kill -INT -- "-$p" 2>/dev/null; done
  sleep 3
  for p in $SN $RN $SIM; do kill -KILL -- "-$p" 2>/dev/null; done
  wait 2>/dev/null
}
trap stop_all EXIT
timeout 300 python3 "$PKG/lib/sim_mujoco/stand_navigation_test.py" --profile "$PROFILE"
RC=$?
echo "stand_navigation_test exit code $RC (logs in $OUT)"
exit $RC
