#!/usr/bin/env bash
# Restart only the sim_overview RViz window (and mp_viz_markers / mp_viz_scene) of a LIVE sim container with the mp_rviz
# files of this checkout, without rebuilding or touching the sim: lighter profile (15 fps, one robot), decimated executed-TCP
# line, and the wall drawn where the sim really has it (Gazebo: the wall scene aligned like the world; MuJoCo: wall_0 of the MJCF).
#
#   docker/tools/relaunch_rviz.sh CONTAINER DOMAIN_ID gazebo WALL_SCENE_JSON PLAN_FILE [STANDOFF]
#   docker/tools/relaunch_rviz.sh CONTAINER DOMAIN_ID mujoco ROBOT_MJCF PLAN_FILE
# Paths are as seen INSIDE the container. Example (the live gazebo-onewall-sovw4 run):
#   docker/tools/relaunch_rviz.sh gz_sim_a 83 gazebo /cp/gazebo-onewall-sovw4_out/data/smoke_wall_scene.json \
#       /cp/gazebo-onewall-sovw4_out/coverage_output.json 0.915
set -euo pipefail
C=${1:?container}; DOM=${2:?ROS_DOMAIN_ID}; SIM=${3:?gazebo|mujoco}; WALL=${4:?wall scene json (gazebo) or robot.xml (mujoco)}
PLAN=${5:-}; STANDOFF=${6:-0.915}
SRC=$(cd "$(dirname "$0")/../.." && pwd)/src
DST=/tmp/mp_rviz_new
docker exec "$C" rm -rf "$DST"; docker exec "$C" mkdir -p "$DST/sim_gazebo"
docker cp "$SRC/common/mp_rviz/." "$C:$DST/"
docker cp "$SRC/simulation/sim_gazebo/sim_gazebo/wall_world.py" "$C:$DST/sim_gazebo/wall_world.py"
if [ "$SIM" = gazebo ]; then
  USE_SIM_TIME=true
  WALLARGS="-p wall_scene_json:=$DST/rviz_world_wall.json"
  PREP="python3 -c \"import sys,json; sys.path.insert(0,'$DST/sim_gazebo'); import wall_world as w; \
s=w.load_shapes('$WALL',standoff=$STANDOFF); json.dump(w.world_wall_scene(s),open('$DST/rviz_world_wall.json','w'))\""
else
  USE_SIM_TIME=false
  WALLARGS="-p wall_mjcf:=$WALL"
  PREP=true
fi
docker exec -e DISPLAY="${DISPLAY:-:0}" -e ROS_DOMAIN_ID="$DOM" "$C" bash -c "
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
pkill -f '[r]viz2' || true; pkill -f '[m]p_viz_markers.py' || true; pkill -f '[m]p_viz_scene.py' || true; sleep 1
$PREP
cd $DST
setsid nohup python3 scripts/mp_viz_markers.py --ros-args -r __node:=mp_viz_markers -p use_sim_time:=$USE_SIM_TIME -p fixed_frame:=odom >/tmp/mp_viz_markers.log 2>&1 &
setsid nohup python3 scripts/mp_viz_scene.py --ros-args -r __node:=mp_viz_scene -p use_sim_time:=$USE_SIM_TIME -p frame_id:=odom \
  -p plan_file:='$PLAN' $WALLARGS >/tmp/mp_viz_scene.log 2>&1 &
setsid nohup rviz2 -d $DST/profiles/sim_overview.rviz --ros-args -r __node:=rviz2_sim_overview -p use_sim_time:=$USE_SIM_TIME >/tmp/rviz2.log 2>&1 &
sleep 3; echo started; pgrep -af 'rviz2|mp_viz' | cut -c1-120"
