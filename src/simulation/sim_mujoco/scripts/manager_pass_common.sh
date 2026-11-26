#!/bin/bash
# Shared steps of the MuJoCo manager passes (run_manager_pass.sh, run_sensed_pass.sh); source it, do not run it.
# Expects the caller to have set (after `set -u`): PKG and SHARE.
#   mp_init dir        output dir and file names (OUT, LOG, REC, COV), empty PIDS
#   mp_profile_init data   robot profile (env MP_PROFILE, default ur10e_telescopic) and the wall data dir: sets MP_PROFILE
#                      (exported), MP_DATA (the dir to use: `data` itself for the default profile, a copy shifted by
#                      shift_wall_scene.py in $OUT/profile_data otherwise), MP_WALL_X, MP_Z_LO, MP_Z_HI, MP_TOOL_YAW,
#                      MP_FREE_POSE and MP_NONDEFAULT (profile_reach.py derives them; docs/sim_mujoco.md, "Running another
#                      robot profile"). Call it after mp_init; returns non-zero if the profile cannot be derived.
#   mp_start name cmd  start a process group, logged to $OUT/name.log (pid kept in PIDS and in MP_LAST)
#   mp_stop_all        trap handler: stop the recorder and every started process group
#   mp_coverage json   coverage plan for the wall scene json into $COV (returns 3 if none)
#   mp_manager_yaml    $OUT/manager.yaml (wall at +x, poses authored for base_footprint at z 1.0..1.3)
#   mp_world_model pcd start world_model_loader_node for the wall cloud (pid in WM)
#   mp_record          start the recorder (pid in REC_PID); with MANAGER_PASS_FILM=1 also render_run.py record
#   mp_run_and_score "$@"  ready pose, manager, stop recording, score; the exit code is left in MP_EXIT
# MP_PROFILE=<name> runs another robot profile (MP_READY_SCALE, MP_FREE_STANDOFF tune its ready move).
# Hooks for another tool kind (run_spray_pass.sh), all optional and unset for the contact passes:
#   MP_COVERAGE_PARAMS     replaces the default `-p swath_m:=.. -p overlap_pct:=..` of mp_coverage
#   MP_MANAGER_YAML_EXTRA  lines appended to manager.yaml (4-space indented parameters)
#   MP_MANAGER_GREP        egrep pattern of the manager log lines echoed after the run
#   MP_SKIP_READY=1        skip ready_pose.py (the manager plans the first move from the stowed pose)
#   MP_READY_ARGS          extra arguments of ready_pose.py (--joints ... --duration ...: another pose than FREE)
#   mp_score               a function that scores $REC instead of the contact score_run call; its status is the score
mp_init() {
  OUT=${1:?output dir}
  mkdir -p "$OUT"
  LOG=$OUT/sim_launch.log
  REC=$OUT/run.csv
  COV=$OUT/coverage_output.json
  PIDS=()
}

mp_profile_init() {
  local src=${1:?wall data dir}
  MP_PROFILE=${MP_PROFILE:-ur10e_telescopic}
  export MP_PROFILE
  MP_DATA=$src
  if [ "$MP_PROFILE" = ur10e_telescopic ]; then
    # the stock wall the passes were written for: nothing derived, nothing shifted
    MP_WORLD_PCD=walls.pcd; MP_NONDEFAULT=0; MP_WALL_X=0.98; MP_Z_LO=1.0; MP_Z_HI=1.3; MP_TOOL_YAW=-90.0; MP_FREE_POSE=
    return 0
  fi
  MP_NONDEFAULT=1
  # the planning scene gets the WHOLE wall (walls_sim.pcd, the cloud of the simulated wall), not only the contact patch
  # (walls.pcd): a planner that cannot see the rest of the wall plans the tool through it (xArm 7, docs/sim_mujoco.md)
  MP_WORLD_PCD=walls_sim.pcd
  if [ -n "${MANAGER_PASS_DATA:-}" ]; then
    echo "MP_PROFILE=$MP_PROFILE with MANAGER_PASS_DATA: a custom wall is not shifted for another profile; unset one of them"
    return 2
  fi
  local vars
  vars=$(python3 "$PKG/lib/sim_mujoco/profile_reach.py" --profile "$MP_PROFILE" --standoff "${MP_FREE_STANDOFF:-0.03}" --shell) || {
    echo "profile $MP_PROFILE is not supported (profile_reach.py failed, message above)" >&2
    return 2
  }
  eval "$vars"
  python3 "$PKG/lib/sim_mujoco/shift_wall_scene.py" "$src" "$OUT/profile_data" "$MP_WALL_DX" 0 0.3 "$MP_WALL_DZ" || return 2
  MP_DATA=$OUT/profile_data
  echo "profile $MP_PROFILE: wall x $MP_WALL_X, z $MP_Z_LO..$MP_Z_HI, tool yaw $MP_TOOL_YAW (derived by profile_reach.py, unconfirmed)"
}

mp_start() {
  local name=$1; shift
  setsid "$@" > "$OUT/$name.log" 2>&1 &
  MP_LAST=$!
  PIDS+=($MP_LAST)
}

mp_stop_all() {
  # the film recorder too: an early exit or Ctrl-C used to leave it running and the final `wait` blocked on it
  [ -n "${FILM_PID:-}" ] && kill -INT "$FILM_PID" 2>/dev/null && wait "$FILM_PID" 2>/dev/null
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
  timeout 120 ros2 run coverage_planner wall_coverage_demo_node --ros-args -p double_pass:=${MANAGER_PASS_DOUBLE_PASS:-false} \
    -p wall_scene_json:=$scene -p output_json:=$COV \
    -p orientation_format:=quaternion -p tool_frame_yaw_deg:=${MANAGER_PASS_TOOL_YAW:-${MP_TOOL_YAW:--90.0}} \
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
    coverage_z_min_m: ${MP_Z_LO:-1.0}
    coverage_z_max_m: ${MP_Z_HI:-1.3}
    coverage_eef_speed_mps: ${MANAGER_PASS_SPEED:-0.01}
    dock_at_stroke_start: true
    dock_standoff_m: ${MANAGER_PASS_STANDOFF:-0.04}
    contact_y_overshoot_m: ${MANAGER_PASS_PRESS:-0.0015}
    target_force_n: ${MANAGER_PASS_TARGET:-10.0}
${MP_MANAGER_YAML_EXTRA:-}
${MANAGER_PASS_PLANNING_CONFIG:+    planning_config: $MANAGER_PASS_PLANNING_CONFIG}
YAML
}

mp_world_model() {
  timeout 60 ros2 run motion_planner_moveit world_model_loader_node --ros-args \
    -p point_cloud_path:=$1 -p frame_id:=base_footprint -p octomap_resolution:=0.01 \
    > "$OUT/world_model.log" 2>&1 &
  WM=$!
  # MANAGER_PASS_CUROBO_WORLD=1: the same cloud into the cuRobo node's world too (the node plans in base_link, 0.31 m
  # above base_footprint, so the cloud is shifted by -0.31 in z). Without it cuRobo plans with an empty world.
  if [ -n "${MANAGER_PASS_CUROBO_WORLD:-}" ]; then
    timeout 60 ros2 run curobo_planner curobo_world_loader --ros-args \
      -p point_cloud_path:=$1 -p frame_id:=base_link -p offset_z:=-0.31 \
      > "$OUT/curobo_world.log" 2>&1 || echo "curobo_world_loader failed (see $OUT/curobo_world.log)"
  fi
}

mp_record() {
  # another profile: the recorder's wall (PATH.wall.json) is the shifted one; MANAGER_PASS_REC_ARGS still wins
  local wall_args=
  [ "${MP_NONDEFAULT:-0}" = 1 ] && wall_args="--wall-x $MP_WALL_X --wall-z $MP_Z_LO $MP_Z_HI"
  python3 "$PKG/lib/sim_mujoco/record_run.py" --record "$REC" --timeout 1200 $wall_args ${MANAGER_PASS_REC_ARGS:-} > "$OUT/record.log" 2>&1 &
  REC_PID=$!
  # MANAGER_PASS_FILM=1: also store the joint states for render_run.py (launch with out_dir:=... to keep the MJCF)
  if [ -n "${MANAGER_PASS_FILM:-}" ]; then
    python3 "$PKG/lib/sim_mujoco/render_run.py" record --out "$OUT/run_film.npz" > "$OUT/film.log" 2>&1 &
    FILM_PID=$!
  fi
}

# Arm collisions the pass score does not see (docs/sim_mujoco.md): the sim logs UNEXPECTED_CONTACT for a new penetrating
# body pair (self collision, an arm link on the wall or an obstacle; the tool on the wall is not listed).
mp_report_contacts() {
  local n
  n=$(grep -c "UNEXPECTED_CONTACT t=" "$OUT/sim_launch.log" 2>/dev/null || true)
  echo "unexpected contacts (new body pairs): ${n:-0}"
  grep -h "UNEXPECTED_CONTACT t=" "$OUT/sim_launch.log" 2>/dev/null | cut -c1-200 | head -10
}

mp_run_and_score() {
  # ready_pose.py is an unplanned joint-space move to the contact FREE pose 3 cm in front of the wall; a non-contact
  # pass sets MP_SKIP_READY=1 and lets the manager plan the first move from the stowed pose instead
  # another profile: the FREE pose is a planned tcp pose from profile_reach.py (ready_pose.py --pose), not UR10e joint angles
  local ready_args=${MP_READY_ARGS:-}
  if [ "${MP_NONDEFAULT:-0}" = 1 ] && [ -z "$ready_args" ]; then
    ready_args="--pose $MP_FREE_POSE --scale ${MP_READY_SCALE:-0.3}"
  fi
  # another profile: reach a pre-pose MP_READY_PRE_M (default 0.15 m) further from the wall first, then the FREE pose: a
  # single planned move from the stowed pose swings the tool down the wall and grazes it with the pad edge (seen on the
  # xArm 7: tilt up to 59 degrees, 57 N, before the manager had started); 0 disables the pre-pose
  if [ "${MP_SKIP_READY:-0}" != 1 ] && [ "${MP_NONDEFAULT:-0}" = 1 ] && [ -z "${MP_READY_ARGS:-}" ] &&
    [ "${MP_READY_PRE_M:-0.15}" != 0 ] && [ -n "${MP_FREE_POSE:-}" ]; then
    local pre
    pre=$(python3 -c 'import sys; v=sys.argv[1:8]; print(" ".join([repr(float(v[0]) - float(sys.argv[8]))] + v[1:]))' \
      $MP_FREE_POSE "${MP_READY_PRE_M:-0.15}") || pre=
    if [ -n "$pre" ]; then
      python3 "$PKG/lib/sim_mujoco/ready_pose.py" --pose $pre --scale "${MP_READY_SCALE:-0.3}" --tol-pos 0.02 --tol-ang 0.15 \
        | tee "$OUT/ready_pose_pre.log"
      if [ "${PIPESTATUS[0]}" != 0 ]; then
        echo "WARNING: the pre-pose move failed (see $OUT/ready_pose_pre.log): the single move to the FREE pose follows and may graze the wall" >&2
      fi
    fi
  fi
  if [ "${MP_SKIP_READY:-0}" != 1 ]; then python3 "$PKG/lib/sim_mujoco/ready_pose.py" $ready_args | tee "$OUT/ready_pose.log"; fi
  timeout 900 ros2 launch manipulation_manager manipulation_manager.launch.py hardware:=jtac_mock use_sim_time:=false \
    profile:=$MP_PROFILE params_file:=$OUT/manager.yaml "$@" > "$OUT/manager.log" 2>&1
  local rc=$?
  echo "manager exit code $rc (logs in $OUT)"
  grep -E "${MP_MANAGER_GREP:-contact success|Coverage plan complete|dock failed|retract failed}" "$OUT/manager.log" | cut -c1-200
  kill -INT "$REC_PID" 2>/dev/null; wait "$REC_PID" 2>/dev/null; REC_PID=
  if [ -n "${FILM_PID:-}" ]; then kill -INT "$FILM_PID" 2>/dev/null; wait "$FILM_PID" 2>/dev/null; cat "$OUT/film.log"; fi
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
  mp_report_contacts
  if [ $rc -ne 0 ]; then MP_EXIT=$rc; else MP_EXIT=$sc; fi
}
