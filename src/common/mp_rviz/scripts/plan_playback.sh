#!/bin/bash
# Plan-only playback: the manipulation manager's real plan (OMPL free-space approach, Cartesian dock / stroke / retract,
# stand navigation) executed on ros2_control MOCK hardware, shown in RViz. No simulator, no physics, no force sensor.
# Usage: plan_playback.sh            (env: PB_OUT host dir, PB_SRC src dir, PB_BUILD_VOL, PB_INSTALL_VOL, PB_DOMAIN, PB_IMAGE)
#   PB_OUT        output/artefact dir (coverage_output.json, data/walls.pcd, ur10e.rm4d are reused if present, else built:
#                 the WHOLE 3 x 2 m wall by default, PB_FULL_WALL=0 = the 0.9 m patch; PB_PLANE_X, PB_SPEED (0.2 m/s), PB_Z_MIN/MAX, PB_MAX_SPAN, PB_SWATH, PB_OVERLAP)
#   PB_SRC        the workspace src mounted at /ws/src of the stack container (default: this checkout's src)
#   PB_BUILD_VOL / PB_INSTALL_VOL  docker volumes with the colcon build / install of that src (default gzcols_b / gzcols_i)
#   PB_RVIZ_WS    a workspace with this package built (build/ install/) for the RViz container (default: PB_OUT/../rv)
# Containers: plan_pb (stack, image motion-planner:sim-gazebo) and plan_pb_rviz (RViz, motion-planner:build); both stay running.
set -u
HERE=$(cd "$(dirname "$0")/.." && pwd)                      # src/common/mp_rviz
ROOT=$(cd "$HERE/../../.." && pwd)
OUT=${PB_OUT:?set PB_OUT to a writable output directory}
SRC=${PB_SRC:-$ROOT/src}
DOM=${PB_DOMAIN:-92}
IMG=${PB_IMAGE:-motion-planner:sim-gazebo}
RVWS=${PB_RVIZ_WS:-$OUT/../rv}
mkdir -p "$OUT"
SLOTS=$ROOT/docker/tools/hostwatch/slots.sh
if [ -x "$SLOTS" ] || [ -f "$SLOTS" ]; then
  bash "$SLOTS" | tee "$OUT/slots.txt"
  grep -q '^node   YES' "$OUT/slots.txt" || { echo "no node slot, not starting"; exit 1; }
fi
xhost +SI:localuser:root >/dev/null 2>&1
docker rm -f plan_pb plan_pb_rviz >/dev/null 2>&1
PBENV=(); for v in $(compgen -e | grep "^PB_"); do PBENV+=(-e "$v=${!v}"); done   # every PB_* setting reaches the container
docker run -d --name plan_pb --ipc=host --net=host --memory=3200m -e ROS_DOMAIN_ID=$DOM "${PBENV[@]}" -e PB_OUT=/out -e PB_MP_RVIZ=/mp_rviz \
  -v "${PB_BUILD_VOL:-gzcols_b}":/ws/build -v "${PB_INSTALL_VOL:-gzcols_i}":/ws/install -v "$SRC":/ws/src \
  -v "$HERE":/mp_rviz:ro -v "$OUT":/out "$IMG" bash /mp_rviz/scripts/plan_playback_inner.sh
# RViz side: plan layers (coverage_output.json of the run), the tool trace from TF tcp_link in odom, live robot
docker run -d --name plan_pb_rviz --ipc=host --net=host --memory=1500m -e ROS_DOMAIN_ID=$DOM -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e DISPLAY=${DISPLAY:-:0} -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix -v "$ROOT":/ws \
  -v "$RVWS/build":/ws/build -v "$RVWS/install":/ws/install -v "$OUT":/out:ro -w /ws motion-planner:build bash -lc "
source /opt/ros/jazzy/setup.bash && source install/setup.bash
until [ -s /out/coverage_output.json ] && [ -s /out/manager.yaml ]; do sleep 3; done; sleep 5
WALL=; [ -s /out/data/smoke_wall_scene.json ] && WALL='-p wall_scene_json:=/out/data/smoke_wall_scene.json'
python3 src/common/mp_rviz/scripts/mp_viz_scene.py --ros-args -p plan_file:=/out/coverage_output.json -p frame_id:=odom -p base_x:=0.0 -p base_y:=0.0 -p arm_base_z:=1.0 \$WALL &
python3 src/common/mp_rviz/scripts/mp_viz_markers.py --ros-args -p fixed_frame:=odom -p force_topic:=/nonexistent_force &
sleep 3; ros2 service call /mp_viz_scene/refresh std_srvs/srv/Trigger &
rviz2 -d src/common/mp_rviz/profiles/layers.rviz -f odom"
echo "running: docker logs -f plan_pb ; logs in $OUT"
