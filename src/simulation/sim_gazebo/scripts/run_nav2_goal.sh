#!/bin/bash
# Nav2 drives the omni base across the empty world past a box (docs/nav2.md). Needs the nav2 image stage, a sourced
# workspace with sim_gazebo, stand_navigator and mobile_manipulator_description installed.
#
#   NAV2_MODE=nav2     NavigateToPose straight to Nav2 (default)
#   NAV2_MODE=adapter  navigate_to_stand with backend:=nav2 (the stand_navigator adapter)
#   NAV2_DEPTH=1       also start the depth camera and feed /depth_sensor/points to the voxel layer
#   NAV2_PARAMS        params file for nav2.launch.py (default: the packaged one)
#   NAV2_GOAL          "x y yaw" goal in odom (default 4 0 0); NAV2_PROFILE: base profile for the adapter
#   NAV2_OUT           log directory (default /tmp/nav2_goal)
set -u
PKG=$(ros2 pkg prefix sim_gazebo)
DESC=$(ros2 pkg prefix mobile_manipulator_description)/share/mobile_manipulator_description
PROFILE=${NAV2_PROFILE:-$DESC/config/profiles/ur10e_telescopic.yaml}
OUT=${NAV2_OUT:-/tmp/nav2_goal}
MODE=${NAV2_MODE:-nav2}
mkdir -p "$OUT"

# empty.sdf plus a static box: centre (2.0, 0), 0.3 x 0.6 m, 0.6 m tall
python3 - "$PKG/share/sim_gazebo/worlds/empty.sdf" "$OUT/box_world.sdf" <<'PY' || exit 2
import sys
sdf = open(sys.argv[1]).read()
box = ('<model name="obstacle_box"><static>true</static><pose>2.0 0 0.3 0 0 0</pose><link name="link">'
       '<collision name="c"><geometry><box><size>0.3 0.6 0.6</size></box></geometry></collision>'
       '<visual name="v"><geometry><box><size>0.3 0.6 0.6</size></box></geometry></visual></link></model>\n')
open(sys.argv[2], 'w').write(sdf.replace('</world>', box + '</world>', 1))
PY

SIM_ARGS=(lidar:=true)
[ "${NAV2_DEPTH:-0}" = 1 ] && SIM_ARGS+=(depth_sensor:=true)
PIDS=()
start() { local n=$1; shift; setsid "$@" > "$OUT/$n.log" 2>&1 & PIDS+=($!); }
stop_all() {
  for p in "${PIDS[@]}"; do kill -INT -- "-$p" 2>/dev/null; done
  sleep 3
  for p in "${PIDS[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done
  pkill -KILL -f "gz sim" 2>/dev/null
  wait 2>/dev/null
}
trap stop_all EXIT
start sim ros2 launch sim_gazebo sim.launch.py profile:=ur10e_telescopic base_motion:=true world:=$OUT/box_world.sdf \
  "${SIM_ARGS[@]}" "$@"
start nav2 ros2 launch sim_gazebo nav2.launch.py ${NAV2_PARAMS:+params_file:=$NAV2_PARAMS}
if [ "$MODE" = adapter ]; then
  start stand_navigator ros2 run stand_navigator stand_navigator_node --ros-args -p base_profile:=$PROFILE -p backend:=nav2
fi
timeout 400 python3 "$PKG/lib/sim_gazebo/nav2_goal_test.py" --mode $MODE ${NAV2_GOAL:+--goal $NAV2_GOAL}
RC=$?
echo "nav2_goal_test exit code $RC (logs in $OUT)"
exit $RC
