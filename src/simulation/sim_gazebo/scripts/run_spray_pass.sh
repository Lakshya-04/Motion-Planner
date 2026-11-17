#!/bin/bash
# Headless NON-CONTACT wall pass in the Gazebo sim: a spray gun held at a stand-off, the real pipeline (the Gazebo twin of
# sim_mujoco/scripts/run_spray_pass.sh, same steps, same seed, same scoring):
#   wall scene -> coverage_planner (tool_name spray_gun: poses at the stand-off, swath from the spray cone)
#   -> coverage_output.json -> manipulation_manager contact:=false (docks, strokes at the stand-off with the
#   valve open, retracts per column through MoveIt + JTAC; no force engage, no press, no wall contact)
# The valve (std_msgs/Bool /tool/valve) is recorded as the `active` column of run.csv; score_run --no-contact counts only
# the valve-open samples and reports the deposition thickness uniformity (CoV). The plane every step uses is the
# one the tcp touches in the contact passes, x = 0.98 in base_footprint (the Gazebo world wall is at 0.915, so the nozzle is
# 6.5 cm further from the real wall than from the plane; nothing sprays in Gazebo, so only the geometry shows).
# Needs a sourced ROS environment with sim_gazebo, ft_conditioner, compliant_trajectory_controller, ft_bridge_hardware,
# coverage_planner, manipulation_manager, motion_planner_moveit and mobile_manipulator_moveit_config installed.
# Extra arguments go to `ros2 launch manipulation_manager`.
#   SPRAY_PASS_OUT (default /tmp/spray_pass)  SPRAY_PASS_STANDOFF (m, 0.30)  SPRAY_PASS_HALF_ANGLE (deg, 20)
#   SPRAY_PASS_OVERLAP (percent, 20)  SPRAY_PASS_SPEED (m/s, 0.05)  SPRAY_PASS_MIN_COVERAGE (0.9)  SPRAY_PASS_MAX_PEAK (N, 0 = none)
#   SPRAY_PASS_READY (seed: planned move to the first stroke's arm configuration, default; none; free = the contact pass's pose)
#   SPRAY_PASS_SEED_Q / SPRAY_PASS_SEED_SCALE (the seed joints in ready_pose.FREE order, velocity scale of the MoveIt plan, default 0.04 = a 27 s plan: it has to be slower than gz's 10 s joint loop, see docs/sim_gazebo.md)
#   SPRAY_PASS_SCORE_ARGS (extra score_run arguments)  SPRAY_PASS_LAUNCH_ARGS
#   GZ_PASS_STILLS=N (colour camera on, N PNG stills into OUT/stills, one every GZ_PASS_STILLS_EVERY s)
set -u
PKG=$(ros2 pkg prefix sim_gazebo)
SHARE=$PKG/share/sim_gazebo
DATA=$SHARE/test/data
. "$PKG/lib/sim_gazebo/gz_pass_common.sh"
STANDOFF=${SPRAY_PASS_STANDOFF:-0.30}        # m, nozzle to wall plane
HALF_ANGLE=${SPRAY_PASS_HALF_ANGLE:-20.0}    # deg, spray cone half-angle (tool_profiles.yaml spray_gun)
OVERLAP=${SPRAY_PASS_OVERLAP:-20.0}          # percent
SPEED=${SPRAY_PASS_SPEED:-0.05}
# effective swath = 0.8 * 2 d tan(theta): the model the planner uses (core_types/sprayer_effective_swath.hpp)
SWATH=$(python3 -c "import math; print(0.8 * 2 * $STANDOFF * math.tan(math.radians($HALF_ANGLE)))")

READY=${SPRAY_PASS_READY:-seed}
READY_ARGS=()
case "$READY" in
  none|free) ;;
  seed) # lift pan shoulder_lift elbow wrist_1 wrist_2 wrist_3 (ready_pose.FREE order); override with SPRAY_PASS_SEED_Q
    READY_ARGS=(--joints "${SPRAY_PASS_SEED_Q:-0.6091,0.1811,0.4325,1.348,1.3607,-1.7519,1.5708}" --planned --scale "${SPRAY_PASS_SEED_SCALE:-0.04}");;
  *) echo "SPRAY_PASS_READY must be seed, none or free"; exit 2;;
esac

gz_init "${SPRAY_PASS_OUT:-/tmp/spray_pass}"
trap gz_stop_all EXIT

# shellcheck disable=SC2046
eval gz_start sim_launch ros2 launch sim_gazebo sim.launch.py $(gz_sim_args) wall_scene:=$DATA/smoke_wall_scene.json \
  standoff:=0.915 ${SPRAY_PASS_LAUNCH_ARGS:-}
gz_start coverage_planner ros2 run coverage_planner coverage_planner_node

# 1. coverage plan for the wall's contact region: the spray_gun profile instead of the contact swath, strokes along z
sleep 3
gz_coverage $DATA/wall_pass_coverage_scene.json \
  "-p tool_name:=spray_gun -p standoff_m:=$STANDOFF -p sweep_direction:=vertical -p overlap_pct:=$OVERLAP" || exit $?

# 2. manager parameters (contact: false)
cat > "$OUT/manager.yaml" <<YAML
manipulation_manager:
  ros__parameters:
    frame_id: base_footprint
    controller_name: combined_arm_lift_controller
    coverage_json_path: $COV
    wall_direction: [1.0, 0.0, 0.0]
    z_offset_m: 0.0
    coverage_z_step_m: 0.05
    coverage_z_min_m: 1.0
    coverage_z_max_m: 1.3
    coverage_eef_speed_mps: $SPEED
    dock_at_stroke_start: true
    dock_standoff_m: 0.04
    contact: false
    valve_topic: /tool/valve
YAML

# 3. the whole wall into MoveIt's scene as an octomap (see run_manager_pass.sh for why it must be the whole wall)
gz_wall_pcd "$OUT/walls.pcd"
gz_world_model "$OUT/walls.pcd" base_footprint

# 4. record, seed the first move, run, score
python3 "$PKG/lib/sim_gazebo/record_run.py" --record "$REC" --timeout 2400 > "$OUT/record.log" 2>&1 &
REC_PID=$!
sleep 40   # gz, controllers and move_group up
wait $WM
gz_stills
case "$READY" in
  seed) python3 "$PKG/lib/sim_gazebo/ready_pose.py" "${READY_ARGS[@]}" | tee "$OUT/ready_pose.log";;
  free) python3 "$PKG/lib/sim_gazebo/ready_pose.py" | tee "$OUT/ready_pose.log";;
esac
timeout 2000 ros2 launch manipulation_manager manipulation_manager.launch.py hardware:=jtac_mock profile:=ur10e_telescopic \
  use_sim_time:=true params_file:=$OUT/manager.yaml "$@" > "$OUT/manager.log" 2>&1
RC=$?
echo "manager exit code $RC (logs in $OUT)"
grep -E "spray success|valve |Coverage plan complete|dock failed|retract failed" "$OUT/manager.log" | cut -c1-200
kill -INT "$REC_PID" 2>/dev/null; wait "$REC_PID" 2>/dev/null; REC_PID=
cat "$OUT/record.log"
SC=0
if [ -f "$REC" ]; then
  # the valve column: some samples spraying, not all of them; the tool never presses on the wall
  awk -F, 'NR>1 {n++; if ($6 == 1) on++; if ($5 > m) m = $5; if ($6 == 1 && $5 > mo) mo = $5} END {printf "recorded %d samples, valve open in %d, peak force %.2f N (%.2f N while spraying)\n", n, on, m, mo}' "$REC"
  ros2 run coverage_planner score_run --no-contact --samples "$REC" --wall "$REC.wall.json" --swath $SWATH \
    --spray-standoff $STANDOFF --spray-half-angle $HALF_ANGLE --min-coverage "${SPRAY_PASS_MIN_COVERAGE:-0.9}" \
    --max-peak "${SPRAY_PASS_MAX_PEAK:-0}" ${SPRAY_PASS_SCORE_ARGS:-}
  SC=$?
  echo "score exit code $SC"
fi
[ $RC -ne 0 ] && exit $RC
exit $SC
