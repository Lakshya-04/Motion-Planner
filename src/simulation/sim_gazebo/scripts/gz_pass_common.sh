#!/bin/bash
# Shared steps of the Gazebo manager passes that are built on it (run_spray_pass.sh, run_multi_stand_pass.sh); source
# it, do not run it. The MuJoCo twin is sim_mujoco/scripts/manager_pass_common.sh (same function names and hooks).
# Expects the caller to have set (after `set -u`): PKG (prefix of sim_gazebo) and DATA (its share/.../test/data).
#   gz_init dir        output dir and file names (OUT, REC, COV), empty PIDS
#   gz_start name cmd  start a process group, logged to $OUT/name.log (pid kept in PIDS and in GZ_LAST)
#   gz_stop_all        trap handler: stop the recorder, the stills and every started process group
#   gz_coverage json params   coverage plan for the wall scene json into $COV (returns 3 if none); the tool yaw is
#                      GZ_TOOL_YAW (default -90, the UR10e ready pose); params are the extra wall_coverage_demo_node arguments
#   gz_wall_pcd file [y0 y1]  the wall plane the tcp touches (x 0.98 in base_footprint) as a 1 cm grid, z 0.6..2.0
#   gz_world_model pcd frame  start world_model_loader_node for the cloud (pid in WM)
#   gz_sim_args        the launch arguments every pass shares (profile, JTAC position only, conditioner, MoveIt);
#                      GZ_POSITION_GAIN adds position_gain:= (gz_ros2_control joint loop, 1/s; unset = plugin default 0.1)
#   gz_stills          GZ_PASS_STILLS=N: also start capture_still.py (N PNGs, one every GZ_PASS_STILLS_EVERY s, default 25)
# The UR meshes (visual and collision) are on by default; GZ_PASS_MESHES=0 turns them off (arm_meshes:=false, the old no-arm-geometry physics).
# GZ_PASS_STILLS makes gz_sim_args add rgb_camera:=true (the camera pose is GZ_PASS_CAMERA_POSE, "x y z roll pitch yaw"
# in base_footprint; GZ_PASS_CAMERA_FRAME=world makes it a static world camera, the pose is then in the world). The Gazebo world is the stock smoke wall at x = 0.915; the plane everything else uses is x = 0.98
# (the tool tip ends 6.5 cm short of the tcp, see docs/sim_gazebo.md).
gz_init() {
  OUT=${1:?output dir}
  mkdir -p "$OUT"
  REC=$OUT/run.csv
  COV=$OUT/coverage_output.json
  PIDS=()
}

gz_start() {
  local name=$1; shift
  setsid "$@" > "$OUT/$name.log" 2>&1 &
  GZ_LAST=$!
  PIDS+=($GZ_LAST)
}

gz_stop_all() {
  [ -n "${REC_PID:-}" ] && kill -INT "$REC_PID" 2>/dev/null && wait "$REC_PID" 2>/dev/null
  [ -n "${STILL_PID:-}" ] && kill -INT "$STILL_PID" 2>/dev/null && wait "$STILL_PID" 2>/dev/null
  for p in "${PIDS[@]}"; do kill -INT -- "-$p" 2>/dev/null; done
  sleep 3
  for p in "${PIDS[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done
  # gz sim (the server) detaches from the process group of its launch file on some images
  pkill -KILL -f "gz sim" 2>/dev/null
  wait 2>/dev/null
}

gz_sim_args() {
  local a="profile:=${GZ_PROFILE:-ur10e_telescopic} controller:=jtac jtac_position_only:=true ft_conditioner:=true payload_mass_kg:=0.8 move_group:=true gui:=false"
  [ "${GZ_PASS_MESHES:-1}" = 0 ] && a="$a arm_meshes:=false"
  [ -n "${GZ_POSITION_GAIN:-}" ] && a="$a position_gain:=$GZ_POSITION_GAIN"
  if [ -n "${GZ_PASS_STILLS:-}" ]; then
    a="$a rgb_camera:=true rgb_camera_frame:=${GZ_PASS_CAMERA_FRAME:-base} rgb_camera_pose:='${GZ_PASS_CAMERA_POSE:-0.0 -2.4 1.6 0 0.11 1.38}'"
  fi
  echo "$a"
}

gz_coverage() {
  local scene=$1 params=${2:-}
  rm -f "$COV"
  timeout 120 ros2 run coverage_planner wall_coverage_demo_node --ros-args -p double_pass:=${GZ_DOUBLE_PASS:-false} \
    -p wall_scene_json:=$scene -p output_json:=$COV \
    -p orientation_format:=quaternion -p tool_frame_yaw_deg:=${GZ_TOOL_YAW:--90.0} \
    $params > "$OUT/coverage_demo.log" 2>&1 &
  local demo=$!
  for _ in $(seq 100); do [ -s "$COV" ] && break; sleep 0.5; done
  kill -INT $demo 2>/dev/null
  if [ ! -s "$COV" ]; then echo "no coverage plan produced (see $OUT/coverage_demo.log)"; return 3; fi
}

gz_wall_pcd() {
  # the scan is the plane the tcp touches, written here instead of shipped as a 600 kB data file
  python3 - "$1" "${2:--1.0}" "${3:-1.0}" <<'PY'
import sys
y0, y1 = float(sys.argv[2]), float(sys.argv[3])
ny = round((y1 - y0) / 0.01) + 1
pts = [(0.98, y0 + i * 0.01, 0.6 + j * 0.01) for i in range(ny) for j in range(141)]
with open(sys.argv[1], 'w') as f:
    f.write("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z label\nSIZE 4 4 4 4\nTYPE F F F U\n"
            f"COUNT 1 1 1 1\nWIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\nDATA ascii\n")
    for x, y, z in pts:
        f.write(f"{x:.3f} {y:.3f} {z:.3f} 0\n")
PY
}

gz_world_model() {
  # $1 cloud for MoveIt, $2 its frame, $3 cloud for cuRobo (default $1; the whole-wall cloud is too big for its box limit)
  timeout 120 ros2 run motion_planner_moveit world_model_loader_node --ros-args \
    -p point_cloud_path:=$1 -p frame_id:=${2:-base_footprint} -p octomap_resolution:=0.01 \
    > "$OUT/world_model.log" 2>&1 &
  WM=$!
  # MANAGER_PASS_CUROBO_WORLD=1: the same cloud into the cuRobo node's world too (the node plans in base_link, 0.31 m
  # above base_footprint, so the cloud is shifted by -0.31 in z). Without it cuRobo plans with an empty world.
  if [ -n "${MANAGER_PASS_CUROBO_WORLD:-}" ]; then
    timeout 60 ros2 run curobo_planner curobo_world_loader --ros-args \
      -p point_cloud_path:=${3:-$1} -p frame_id:=base_link -p offset_z:=-0.31 \
      > "$OUT/curobo_world.log" 2>&1 || echo "curobo_world_loader failed (see $OUT/curobo_world.log)"
  fi
}

gz_stills() {
  [ -z "${GZ_PASS_STILLS:-}" ] && return 0
  python3 "$PKG/lib/sim_gazebo/capture_still.py" "${GZ_PASS_STILLS_DIR:-$OUT/stills}" --count "$GZ_PASS_STILLS" \
    --every "${GZ_PASS_STILLS_EVERY:-25}" --prefix "${GZ_PASS_STILLS_PREFIX:-still}" > "$OUT/stills.log" 2>&1 &
  STILL_PID=$!
}
