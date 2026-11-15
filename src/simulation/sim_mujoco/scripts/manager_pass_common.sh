#!/bin/bash
# Shared steps of the MuJoCo manager passes (run_manager_pass.sh, run_sensed_pass.sh); source it, do not run it.
# Expects the caller to have set (after `set -u`): PKG and SHARE.
#   mp_init dir        output dir and file names (OUT, LOG, REC, COV), empty PIDS
#   mp_start name cmd  start a process group, logged to $OUT/name.log (pid kept in PIDS and in MP_LAST)
#   mp_stop_all        trap handler: stop the recorder and every started process group
#   mp_coverage json   coverage plan for the wall scene json into $COV (returns 3 if none)
#   mp_manager_yaml    $OUT/manager.yaml (wall at +x, poses authored for base_footprint at z 1.0..1.3)
#   mp_world_model pcd start world_model_loader_node for the wall cloud (pid in WM)
#   mp_record          start the recorder (pid in REC_PID)
#   mp_run_and_score "$@"  ready pose, manager, stop recording, score; the exit code is left in MP_EXIT
# Hooks for another tool kind (run_spray_pass.sh), all optional and unset for the contact passes:
#   MP_COVERAGE_PARAMS     replaces the default `-p swath_m:=.. -p overlap_pct:=..` of mp_coverage
#   MP_MANAGER_YAML_EXTRA  lines appended to manager.yaml (4-space indented parameters)
#   MP_MANAGER_GREP        egrep pattern of the manager log lines echoed after the run
#   mp_score               a function that scores $REC instead of the contact score_run call; its status is the score
mp_init() {
  OUT=${1:?output dir}
  mkdir -p "$OUT"
  LOG=$OUT/sim_launch.log
  REC=$OUT/run.csv
  COV=$OUT/coverage_output.json
  PIDS=()
}

mp_start() {
  local name=$1; shift
  setsid "$@" > "$OUT/$name.log" 2>&1 &
  MP_LAST=$!
  PIDS+=($MP_LAST)
}

mp_stop_all() {
  [ -n "${REC_PID:-}" ] && kill -INT "$REC_PID" 2>/dev/null && wait "$REC_PID" 2>/dev/null
  for p in "${PIDS[@]}"; do kill -INT -- "-$p" 2>/dev/null; done
  sleep 3
  for p in "${PIDS[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done
  wait 2>/dev/null
}

mp_coverage() {
  local scene=$1 params
  if [ "${MP_COVERAGE_PARAMS+set}" = set ]; then
    params=$MP_COVERAGE_PARAMS
  else
    params="-p swath_m:=${MANAGER_PASS_SWATH:-0.14} -p overlap_pct:=${MANAGER_PASS_OVERLAP:-15.0}"
  fi
  rm -f "$COV"
  timeout 120 ros2 run coverage_planner wall_coverage_demo_node --ros-args \
    -p wall_scene_json:=$scene -p output_json:=$COV \
    -p orientation_format:=quaternion -p tool_frame_yaw_deg:=${MANAGER_PASS_TOOL_YAW:--90.0} \
    $params \
    > "$OUT/coverage_demo.log" 2>&1 &
  local demo=$!
  for _ in $(seq 100); do [ -s "$COV" ] && break; sleep 0.5; done
  kill -INT $demo 2>/dev/null
  if [ ! -s "$COV" ]; then echo "no coverage plan produced (see $OUT/coverage_demo.log)"; return 3; fi
}

mp_manager_yaml() {
  cat > "$OUT/manager.yaml" <<YAML
manipulation_manager:
  ros__parameters:
    frame_id: base_footprint
    coverage_json_path: $COV
    wall_direction: [1.0, 0.0, 0.0]
    z_offset_m: 0.0
    coverage_z_step_m: 0.05
    coverage_z_min_m: 1.0
    coverage_z_max_m: 1.3
    coverage_eef_speed_mps: ${MANAGER_PASS_SPEED:-0.01}
    dock_at_stroke_start: true
    dock_standoff_m: ${MANAGER_PASS_STANDOFF:-0.04}
    contact_y_overshoot_m: ${MANAGER_PASS_PRESS:-0.0015}
    target_force_n: ${MANAGER_PASS_TARGET:-10.0}
${MP_MANAGER_YAML_EXTRA:-}
YAML
}

mp_world_model() {
  timeout 60 ros2 run motion_planner_moveit world_model_loader_node --ros-args \
    -p point_cloud_path:=$1 -p frame_id:=base_footprint -p octomap_resolution:=0.01 \
    > "$OUT/world_model.log" 2>&1 &
  WM=$!
}

mp_record() {
  python3 "$PKG/lib/sim_mujoco/record_run.py" --record "$REC" --timeout 1200 > "$OUT/record.log" 2>&1 &
  REC_PID=$!
}

mp_run_and_score() {
  python3 "$PKG/lib/sim_mujoco/ready_pose.py" | tee "$OUT/ready_pose.log"
  timeout 900 ros2 launch manipulation_manager manipulation_manager.launch.py hardware:=jtac_mock use_sim_time:=false \
    params_file:=$OUT/manager.yaml "$@" > "$OUT/manager.log" 2>&1
  local rc=$?
  echo "manager exit code $rc (logs in $OUT)"
  grep -E "${MP_MANAGER_GREP:-contact success|Coverage plan complete|dock failed|retract failed}" "$OUT/manager.log" | cut -c1-200
  kill -INT "$REC_PID" 2>/dev/null; wait "$REC_PID" 2>/dev/null; REC_PID=
  cat "$OUT/record.log"
  local sc=0
  if [ -f "$REC" ] && declare -F mp_score > /dev/null; then
    mp_score
    sc=$?
    echo "score exit code $sc"
  elif [ -f "$REC" ]; then
    ros2 run coverage_planner score_run --samples "$REC" --wall "$REC.wall.json" --target-force "${MANAGER_PASS_TARGET:-10}" \
      --band 1.5 --swath ${MANAGER_PASS_SWATH:-0.14} --min-coverage "${MANAGER_PASS_MIN_COVERAGE:-0.9}" ${MANAGER_PASS_SCORE_ARGS:-}
    sc=$?
    echo "score exit code $sc"
  fi
  if [ $rc -ne 0 ]; then MP_EXIT=$rc; else MP_EXIT=$sc; fi
}
