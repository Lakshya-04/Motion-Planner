#!/bin/bash
# Closed loop in the MuJoCo sim for a wall that needs SEVERAL stands: the manager is an action server
# (run_on_start:=false) and one `execute_coverage` goal (multi_stand) does everything:
#   plan_stands for the whole plan -> columns grouped per stand -> for each stand: stow the arm, drive there
#   (navigate_to_stand), run its columns in the world (odom) frame -> next stand.
# The wall is MULTI_STAND_WALL_WIDTH wide (default 0.9 m, the stock patch is 0.3) starting at y = MULTI_STAND_WALL_DY,
# at x = 0.98 + MULTI_STAND_WALL_DX (shift_wall_scene.py). Run, record in odom and score like run_stand_pass.sh.
#   MULTI_STAND_SCENARIO=name      a wall from make_scenario_scene.py (size, position and obstacles) instead of the DX/DY/WIDTH ones
#   MULTI_STAND_PATCH_Z="lo,hi"    contact patch height in m (needs MULTI_STAND_SCENARIO; the wall grows 0.3 m above and below it);
#                                  the scenario's own patch is used otherwise (two_panels_tall: 0.6,2.0); sets MP_Z_LO / MP_Z_HI,
#                                  the manager's coverage z range and the recorder's wall band
#   MULTI_STAND_COVERAGE_EXTRA     more ros arguments of the coverage demo (-p obstacle_inflation_radius_m:=0.07)
#   MULTI_STAND_REACH_FRACTION=0.7       keep the tool within this fraction of MULTI_STAND_ARM_REACH (default 1.3 m, UR10e) by moving the lift (needs DOCK_IK_CONTINUITY=true; default 0 = off)
#   MULTI_STAND_MIN_CLEARANCE=0.5        stand planner floor on the chassis-to-wall clearance in m (default 0 = off)
#   MULTI_STAND_DOCK_IK_CONTINUITY=true  choose the dock joint goal among IK solutions by continuity (manager `dock_ik_continuity`, default false)
#   MULTI_STAND_DOCK_SEED='[...]'  seed of the dock IK (manager `dock_seed_joint_state`, lift first); the default is a UR10e-on-lift
#                                  configuration docked at the wall, [] chains the seeds from the previous dock
#   MULTI_STAND_MAX_SPAN=0.5       widest stretch of wall one stand is trusted with (manager `stand_max_span_m`)
#   MULTI_STAND_CANCEL_AFTER_S=N   cancel the goal N s after it was accepted instead (checks the arm stopped)
#   MULTI_STAND_MANAGER_EXTRA      extra manager.yaml lines (4-space indented), e.g. `stand_nav_timeout_s: 2.0` to make
#                                  the navigation fail and watch the skip
#   MP_PROFILE=<name>              another robot profile (STAND_PROFILE, if set, still wins; default ur10e_telescopic): the wall x
#                                  and z band, the tool roll, the reachability map's arm, the dock seed (UR10e only: [] elsewhere,
#                                  MULTI_STAND_DOCK_SEED overrides) and the arm mount height follow the profile
#                                  (profile_reach.py, docs/sim_mujoco.md "Running another robot profile"; unconfirmed in a sim)
#   MANAGER_PASS_SCORE_ARGS / MULTI_STAND_SCORE_ARGS  extra score_run arguments
# Needs a sourced ROS environment with sim_mujoco, reachability, stand_navigator, ft_conditioner,
# compliant_trajectory_controller, coverage_planner, manipulation_manager, coverage_task_msgs and
# mobile_manipulator_moveit_config. Extra arguments go to `ros2 launch manipulation_manager`.
# MANAGER_PASS_PLANNING_CONFIG=/path/planning_curobo.yaml: the manager builds its planners from that file (planning_config
# parameter, written into manager.yaml; unset = the packaged planning.yaml). Needs the cuRobo node: docs/sim_*.md.
set -u
PKG=$(ros2 pkg prefix sim_mujoco)
SHARE=$PKG/share/sim_mujoco
DESC=$(ros2 pkg prefix mobile_manipulator_description)/share/mobile_manipulator_description
STAND_PROFILE=${STAND_PROFILE:-${MP_PROFILE:-ur10e_telescopic}}
MP_PROFILE=$STAND_PROFILE
export MP_PROFILE
PROFILE=$DESC/config/profiles/$STAND_PROFILE.yaml
# stock wall and UR10e values, replaced by profile_reach.py for another profile
MP_WALL_DX=0.0; MP_WALL_DZ=0.0; MP_Z_LO=1.0; MP_Z_HI=1.3; MP_TOOL_YAW=-90.0; MP_ARM=ur10e; MP_ARM_MOUNT_Z=1.17; MP_LIFT_STROKE=1.5
MP_TCP_XYZ=; DOCK_SEED='[0.1, 2.79, -2.80, -1.30, -2.20, -0.36, -1.5708]'
if [ "$STAND_PROFILE" != ur10e_telescopic ]; then
  vars=$(python3 "$PKG/lib/sim_mujoco/profile_reach.py" --profile "$PROFILE" --shell) || {
    echo "run_multi_stand_pass: profile $STAND_PROFILE is not supported (profile_reach.py failed, message above); not running with the UR10e defaults" >&2
    exit 2
  }
  eval "$vars"
  DOCK_SEED='[]'   # the default seed is a UR10e-on-lift configuration
fi
OUT=${MULTI_STAND_OUT:-/tmp/multi_stand_pass}
DX=${MULTI_STAND_WALL_DX:-0.0}
DY=${MULTI_STAND_WALL_DY:-0.3}
WIDTH=${MULTI_STAND_WALL_WIDTH:-0.9}
CANCEL_AFTER=${MULTI_STAND_CANCEL_AFTER_S:-0}
mkdir -p "$OUT"
COV=$OUT/coverage_output.json
REC=$OUT/run.csv

# MULTI_STAND_SCENARIO=name takes the wall and its obstacles from make_scenario_scene.py (it sets DX, DY and WIDTH)
if [ -n "${MULTI_STAND_SCENARIO:-}" ]; then
  [ "$STAND_PROFILE" != ur10e_telescopic ] && { echo "MULTI_STAND_SCENARIO walls are placed for ur10e_telescopic; not shifted for $STAND_PROFILE"; exit 2; }
  PATCH_ARGS=(); [ -n "${MULTI_STAND_PATCH_Z:-}" ] && PATCH_ARGS=(--patch-z "$MULTI_STAND_PATCH_Z")
  eval "$(python3 "$PKG/lib/sim_mujoco/make_scenario_scene.py" ${PATCH_ARGS[@]+"${PATCH_ARGS[@]}"} "$SHARE/data" "$OUT/data" "$MULTI_STAND_SCENARIO")" || exit 2
  MP_Z_LO=$PATCH_Z_LO; MP_Z_HI=$PATCH_Z_HI   # the scenario's patch height (stock 1.0..1.3 unless it is a tall wall)
else
  [ -n "${MULTI_STAND_PATCH_Z:-}" ] && { echo "MULTI_STAND_PATCH_Z needs MULTI_STAND_SCENARIO (make_scenario_scene.py builds the tall wall)"; exit 2; }
  python3 "$PKG/lib/sim_mujoco/shift_wall_scene.py" "$SHARE/data" "$OUT/data" "$(python3 -c "print(round($DX + $MP_WALL_DX, 3))")" "$DY" "$WIDTH" "$MP_WALL_DZ" || exit 2
fi
WALL_X=$(python3 -c "print(round(0.98 + $DX + $MP_WALL_DX, 3))")
WALL_Y0=$(python3 -c "print($DY)")
WALL_Y1=$(python3 -c "print($WIDTH + $DY)")

MAP=${STAND_MAP:-}
if [ -z "$MAP" ]; then
  MAP=$OUT/$MP_ARM.rm4d
  RE=$(ros2 pkg prefix reachability)
  XACRO_EXTRA=()
  [ -n "$MP_TCP_XYZ" ] && XACRO_EXTRA=(tcp_xyz:="$MP_TCP_XYZ" lift_stroke:=$MP_LIFT_STROKE)   # another profile's tool offset and stroke
  xacro "$DESC/urdf/mobile_manipulator.urdf.xacro" arm:=$MP_ARM ${XACRO_EXTRA[@]+"${XACRO_EXTRA[@]}"} -o "$OUT/robot.urdf" || exit 2
  python3 "$RE/lib/reachability/urdf_to_chain.py" "$OUT/robot.urdf" --base arm_base_link --tip tcp_link \
    -o "$OUT/${MP_ARM}_chain.yaml" || exit 2
  "$RE/lib/reachability/rm4d_tool" build "$OUT/${MP_ARM}_chain.yaml" "$MAP" --samples 3000000 --resolution 0.05 \
    --extent 2.0 --zmin -1.8 --zmax 2.0 > "$OUT/map_build.log" 2>&1 || { cat "$OUT/map_build.log"; exit 2; }
fi

PIDS=()
start() { local name=$1; shift; setsid "$@" > "$OUT/$name.log" 2>&1 & PIDS+=($!); }
stop_all() {
  # the film recorder too: an early exit or Ctrl-C used to leave it running and the final `wait` blocked on it
  [ -n "${FILM_PID:-}" ] && kill -INT "$FILM_PID" 2>/dev/null && wait "$FILM_PID" 2>/dev/null
  [ -n "${REC_PID:-}" ] && kill -INT "$REC_PID" 2>/dev/null && wait "$REC_PID" 2>/dev/null
  for p in "${PIDS[@]}"; do kill -INT -- "-$p" 2>/dev/null; done
  sleep 3
  for p in "${PIDS[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done
  wait 2>/dev/null
}
trap stop_all EXIT

start sim_launch ros2 launch sim_mujoco sim_mujoco.launch.py profile:=$STAND_PROFILE controller:=jtac \
  conditioner:=true payload_mass_kg:=0.5 move_group:=true base_motion:=true scene:=$OUT/data/smoke_wall_scene.json \
  ${MULTI_STAND_LAUNCH_ARGS:-}
# the demo rasterizes the obstacle clearance into the grid itself; the service node would add its own 0.3 m default
start coverage_planner ros2 run coverage_planner coverage_planner_node --ros-args -p obstacle_inflation_radius:=0.0
start reachability ros2 run reachability reachability_node --ros-args -r __node:=reachability_node -p map_file:=$MAP \
  -p base_profile:=$PROFILE -p stand_min_clearance_m:=${MULTI_STAND_MIN_CLEARANCE:-0.0}
start stand_navigator ros2 run stand_navigator stand_navigator_node --ros-args -p base_profile:=$PROFILE

# 1. coverage plan for the wall's contact region, in the world frame
rm -f "$COV"
sleep 3
timeout 120 ros2 run coverage_planner wall_coverage_demo_node --ros-args \
  -p wall_scene_json:=$OUT/data/wall_pass_coverage_scene.json -p output_json:=$COV \
  -p orientation_format:=quaternion -p tool_frame_yaw_deg:=${MULTI_STAND_TOOL_YAW:-$MP_TOOL_YAW} \
  -p swath_m:=${MULTI_STAND_SWATH:-0.14} -p overlap_pct:=${MULTI_STAND_OVERLAP:-15.0} ${MULTI_STAND_COVERAGE_EXTRA:-} > "$OUT/coverage_demo.log" 2>&1 &
DEMO=$!
for _ in $(seq 100); do [ -s "$COV" ] && break; sleep 0.5; done
kill -INT $DEMO 2>/dev/null
if [ ! -s "$COV" ]; then echo "no coverage plan produced (see $OUT/coverage_demo.log)"; exit 3; fi

# 2. manager parameters: an action server, the plan stays in odom
# MANAGER_PASS_CUROBO_WORLD=1: every stand arrival also pushes the wall into the cuRobo node's world, posed from TF
# (source_frame odom -> base_link), so cuRobo sees the wall relative to the base at each stand.
CUROBO_ARRIVAL=""
if [ -n "${MANAGER_PASS_CUROBO_WORLD:-}" ]; then
  CUROBO_ARRIVAL="; ros2 run curobo_planner curobo_world_loader --ros-args -p point_cloud_path:=$OUT/data/walls_sim.pcd -p frame_id:=base_link -p source_frame:=odom >> $OUT/curobo_world.log 2>&1"
fi

cat > "$OUT/manager.yaml" <<YAML
manipulation_manager:
  ros__parameters:
    run_on_start: false
    frame_id: base_footprint
    world_frame: odom
    coverage_json_path: $COV
    wall_direction: [1.0, 0.0, 0.0]
    z_offset_m: 0.0
    coverage_z_step_m: 0.05
    coverage_z_min_m: $MP_Z_LO
    coverage_z_max_m: $MP_Z_HI
    coverage_eef_speed_mps: ${MULTI_STAND_SPEED:-0.01}
    dock_at_stroke_start: true
    stand_max_span_m: ${MULTI_STAND_MAX_SPAN:-0.5}
    stand_arrival_command: 'ros2 run motion_planner_moveit world_model_loader_node --ros-args -p point_cloud_path:=$OUT/data/walls_sim.pcd -p frame_id:=odom -p octomap_resolution:=0.01 >> $OUT/world_model.log 2>&1${CUROBO_ARRIVAL}'
    wall_clear_command: 'ros2 run motion_planner_moveit world_model_loader_node --ros-args -p clear:=true >> $OUT/world_model.log 2>&1'
    dock_attempts: ${MULTI_STAND_DOCK_ATTEMPTS:-6}
    dock_seed_joint_state: ${MULTI_STAND_DOCK_SEED:-$DOCK_SEED}
    dock_ik_continuity: ${MULTI_STAND_DOCK_IK_CONTINUITY:-false}
    dock_ik_reach_fraction: ${MULTI_STAND_REACH_FRACTION:-0.0}
    dock_ik_reach_m: ${MULTI_STAND_ARM_REACH:-1.3}
    dock_ik_lift_joint: 0
    dock_ik_lift_min_m: 0.0
    dock_ik_lift_max_m: ${MULTI_STAND_LIFT_STROKE:-1.5}
    dock_approach_extra_m: ${MULTI_STAND_APPROACH_EXTRA:-0.1}
    dock_standoff_m: ${MULTI_STAND_STANDOFF:-0.04}
    contact_y_overshoot_m: ${MULTI_STAND_PRESS:-0.0015}
    target_force_n: ${MULTI_STAND_TARGET:-10.0}
    base_check: "off"
    base_check_service: /reachability_node/plan_stands
    base_check_lift_max_m: $MP_LIFT_STROKE
    base_check_arm_mount_z_m: $MP_ARM_MOUNT_Z
${MULTI_STAND_MANAGER_EXTRA:-}
${MANAGER_PASS_PLANNING_CONFIG:+    planning_config: $MANAGER_PASS_PLANNING_CONFIG}
${MANAGER_PASS_PLANNING_CONFIG:+    seed_start_from_live_state: true}
YAML

python3 "$PKG/lib/sim_mujoco/record_run.py" --record "$REC" --timeout 3000 --frame odom --wall-x "$WALL_X" \
  --wall-y $WALL_Y0 $WALL_Y1 --wall-z $MP_Z_LO $MP_Z_HI --scene "$OUT/data/wall_pass_coverage_scene.json" > "$OUT/record.log" 2>&1 &
REC_PID=$!
# MULTI_STAND_FILM=1: also store the joint states for render_run.py (launch with out_dir:=... in MULTI_STAND_LAUNCH_ARGS)
if [ -n "${MULTI_STAND_FILM:-}" ]; then
  python3 "$PKG/lib/sim_mujoco/render_run.py" record --out "$OUT/run_film.npz" > "$OUT/film.log" 2>&1 &
  FILM_PID=$!
fi
sleep 25   # sim, controllers and move_group up

# 3. the wall into MoveIt's scene (world frame; TF places it relative to base_footprint): walls_sim.pcd, the sim wall's
#    whole extent (walls.pcd is the contact patch only, MoveIt would not know the wall's margin the arm can land on)
timeout 120 ros2 run motion_planner_moveit world_model_loader_node --ros-args \
  -p point_cloud_path:=$OUT/data/walls_sim.pcd -p frame_id:=odom -p octomap_resolution:=0.01 \
  > "$OUT/world_model.log" 2>&1

# 4. the manager as an action server, one goal
start manager ros2 launch manipulation_manager manipulation_manager.launch.py hardware:=jtac_mock use_sim_time:=false \
  params_file:=$OUT/manager.yaml "$@"
CLIENT_ARGS="--multi-stand --coverage $COV"
[ "$CANCEL_AFTER" != 0 ] && CLIENT_ARGS="$CLIENT_ARGS --cancel-after $CANCEL_AFTER"
timeout 3000 python3 -u "$PKG/lib/sim_mujoco/execute_coverage_client.py" $CLIENT_ARGS | tee "$OUT/client.log"
RC=${PIPESTATUS[0]}
echo "client exit code $RC (logs in $OUT)"
grep -E "stand [0-9]+:|arrived|not reached|contact success|dock failed|retract failed|stow|canceled|no stand" "$OUT/manager.log" | cut -c1-200
if [ "$CANCEL_AFTER" != 0 ]; then
  echo "admittance axes after the cancel: $(ros2 param get --spin-time 5 /combined_arm_lift_controller admittance.selected_axes 2>&1 | tr '\n' ' ')"
  exit $RC
fi
kill -INT "$REC_PID" 2>/dev/null; wait "$REC_PID" 2>/dev/null; REC_PID=
if [ -n "${FILM_PID:-}" ]; then kill -INT "$FILM_PID" 2>/dev/null; wait "$FILM_PID" 2>/dev/null; cat "$OUT/film.log"; fi
cat "$OUT/record.log"
# lift range, chassis-to-wall clearance, tool reach fraction (a report; run_metrics.py RUN_DIR prints it again)
echo "run metrics:"
python3 "$PKG/lib/sim_mujoco/run_metrics.py" "$OUT" --profile "$PROFILE" || echo "run_metrics failed (not a gate)"
SC=0
if [ -f "$REC" ]; then
  ros2 run coverage_planner score_run --samples "$REC" --wall "$REC.wall.json" --target-force "${MULTI_STAND_TARGET:-10}" \
    --band 1.5 --swath ${MULTI_STAND_SWATH:-0.14} --min-coverage "${MULTI_STAND_MIN_COVERAGE:-0.9}" ${MANAGER_PASS_SCORE_ARGS:-} ${MULTI_STAND_SCORE_ARGS:-}
  SC=$?
  echo "score exit code $SC"
fi
[ $RC -ne 0 ] && exit $RC
exit $SC
