#!/bin/bash
# Obstacle stop from the simulated lidar, Gazebo. Sourced ROS environment with sim_gazebo, stand_navigator and
# mobile_manipulator_description installed. Empty world plus a floor-standing box 1.6 m ahead of the base; the base is
# sent 3 m straight ahead through navigate_to_stand. No reachability or stand planning here (the MuJoCo script does that).
#
#   LIDAR_STOP_LIDAR=1   lidar:=true and obstacle_scan_topic:=/scan (default); 0: no lidar, the base is expected to hit the box
#   LIDAR_STOP_STOP_M    obstacle_stop_m (default 0.30), LIDAR_STOP_HOLD_S obstacle_hold_s (default 2.0)
#   LIDAR_STOP_OUT       log directory (default /tmp/lidar_stop_gz)
set -u
PKG=$(ros2 pkg prefix sim_gazebo)
DESC=$(ros2 pkg prefix mobile_manipulator_description)/share/mobile_manipulator_description
PROFILE=$DESC/config/profiles/ur10e_telescopic.yaml
OUT=${LIDAR_STOP_OUT:-/tmp/lidar_stop_gz}
LIDAR=${LIDAR_STOP_LIDAR:-1}
STOP_M=${LIDAR_STOP_STOP_M:-0.30}
HOLD_S=${LIDAR_STOP_HOLD_S:-2.0}
mkdir -p "$OUT"

# empty.sdf plus a static box: centre x 1.6, 0.3 m long, 0.6 m wide, 0.6 m tall (the lidar plane is at 0.2 m)
python3 - "$PKG/share/sim_gazebo/worlds/empty.sdf" "$OUT/box_world.sdf" <<'PY' || exit 2
import sys
sdf = open(sys.argv[1]).read()
box = ('<model name="obstacle_box"><static>true</static><pose>1.6 0 0.3 0 0 0</pose><link name="link">'
       '<collision name="c"><geometry><box><size>0.3 0.6 0.6</size></box></geometry></collision>'
       '<visual name="v"><geometry><box><size>0.3 0.6 0.6</size></box></geometry></visual></link></model>\n')
open(sys.argv[2], 'w').write(sdf.replace('</world>', box + '</world>', 1))
PY

SIM_ARGS=()
NAV_ARGS=()
if [ "$LIDAR" = 1 ]; then
  SIM_ARGS+=(lidar:=true)
  NAV_ARGS+=(-p obstacle_scan_topic:=/scan -p obstacle_stop_m:=$STOP_M -p obstacle_hold_s:=$HOLD_S)
  MODE=with
else
  MODE=without
fi
setsid ros2 launch sim_gazebo sim.launch.py profile:=ur10e_telescopic base_motion:=true world:=$OUT/box_world.sdf \
  "${SIM_ARGS[@]}" "$@" > "$OUT/sim_launch.log" 2>&1 &
SIM=$!
setsid ros2 run stand_navigator stand_navigator_node --ros-args -p base_profile:=$PROFILE "${NAV_ARGS[@]}" \
  > "$OUT/stand_navigator.log" 2>&1 &
SN=$!
stop_all() {
  for p in $SN $SIM; do kill -INT -- "-$p" 2>/dev/null; done
  sleep 3
  for p in $SN $SIM; do kill -KILL -- "-$p" 2>/dev/null; done
  wait 2>/dev/null
}
trap stop_all EXIT
timeout 300 python3 "$PKG/lib/sim_gazebo/lidar_stop_test.py" --mode $MODE --stop-m $STOP_M
RC=$?
echo "lidar_stop_test exit code $RC (logs in $OUT)"
exit $RC
