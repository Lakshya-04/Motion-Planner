#!/bin/bash
# Headless NON-CONTACT wall pass in the MuJoCo sim: a spray gun held at a stand-off, the real pipeline:
#   wall scene -> coverage_planner (tool_name spray_gun: poses at the stand-off, swath from the spray cone)
#   -> coverage_output.json -> manipulation_manager contact:=false (docks, strokes at the stand-off with the
#   valve open, retracts per column through MoveIt + JTAC; no force engage, no press, no wall contact)
# The valve (std_msgs/Bool /tool/valve) is recorded as the `active` column of run.csv; score_run --no-contact
# counts only the valve-open samples and reports the deposition thickness uniformity (CoV). Same steps as
# run_manager_pass.sh (shared in manager_pass_common.sh, which stays the contact version). Needs a sourced ROS
# environment with sim_mujoco, ft_conditioner, compliant_trajectory_controller, coverage_planner, manipulation_manager
# and mobile_manipulator_moveit_config installed. Extra arguments go to `ros2 launch manipulation_manager`.
#   SPRAY_PASS_OUT (default /tmp/spray_pass)  SPRAY_PASS_STANDOFF (m, 0.30)  SPRAY_PASS_HALF_ANGLE (deg, 20)
#   SPRAY_PASS_OVERLAP (percent, 20)  SPRAY_PASS_SPEED (m/s, 0.05)  SPRAY_PASS_MIN_COVERAGE (0.9)  SPRAY_PASS_MAX_PEAK (N, 0 = no force limit)
#   SPRAY_PASS_READY (seed: slow move to the first stroke's arm configuration, default; none; free = the contact passes' move 3 cm from the wall)
#   SPRAY_PASS_SEED_Q / SPRAY_PASS_SEED_SCALE (the seed joints in smoke_test.FREE order, velocity scale 0.3 of the MoveIt plan)
#   SPRAY_PASS_SCORE_ARGS (extra score_run arguments, e.g. "--max-thickness-cov 0.6")  SPRAY_PASS_LAUNCH_ARGS
set -u
PKG=$(ros2 pkg prefix sim_mujoco)
SHARE=$PKG/share/sim_mujoco
. "$PKG/lib/sim_mujoco/manager_pass_common.sh"
STANDOFF=${SPRAY_PASS_STANDOFF:-0.30}        # m, nozzle to wall
HALF_ANGLE=${SPRAY_PASS_HALF_ANGLE:-20.0}    # deg, spray cone half-angle (tool_profiles.yaml spray_gun)
OVERLAP=${SPRAY_PASS_OVERLAP:-20.0}          # percent
# effective swath = 0.8 * 2 d tan(theta): the model the planner uses (core_types/sprayer_effective_swath.hpp)
SWATH=$(python3 -c "import math; print(0.8 * 2 * $STANDOFF * math.tan(math.radians($HALF_ANGLE)))")

# the spray_gun profile instead of the contact swath: poses at the stand-off, strokes along z (manager columns)
MP_COVERAGE_PARAMS="-p tool_name:=spray_gun -p standoff_m:=$STANDOFF -p sweep_direction:=vertical -p overlap_pct:=$OVERLAP"
MANAGER_PASS_SPEED=${SPRAY_PASS_SPEED:-0.05}
MP_MANAGER_YAML_EXTRA=$'    contact: false\n    valve_topic: /tool/valve'
# First move: SPRAY_PASS_READY=seed (default) is a MoveIt-planned (collision-checked) move to the arm configuration of the first stroke (tool 0.3 m from
# the wall, lift raised); none leaves the first move to the manager from the stowed pose (an elbow-flipped solution with the lift
# unused: shorter strokes); free is the contact passes' unplanned move to the FREE pose 3 cm from the wall (force spikes).
# SPRAY_PASS_SKIP_READY=1 / 0 is the older switch for none / free.
READY=${SPRAY_PASS_READY:-}
[ -z "$READY" ] && case "${SPRAY_PASS_SKIP_READY:-}" in 1) READY=none;; 0) READY=free;; *) READY=seed;; esac
MP_SKIP_READY=0
case "$READY" in
  none) MP_SKIP_READY=1;;
  free) ;;
  seed) # lift pan shoulder_lift elbow wrist_1 wrist_2 wrist_3 (smoke_test.FREE order); override with SPRAY_PASS_SEED_Q
    MP_READY_ARGS="--joints ${SPRAY_PASS_SEED_Q:-0.6091,0.1811,0.4325,1.348,1.3607,-1.7519,1.5708} --planned --scale ${SPRAY_PASS_SEED_SCALE:-0.3}";;
  *) echo "SPRAY_PASS_READY must be seed, none or free"; exit 2;;
esac
MP_MANAGER_GREP="spray success|valve |Coverage plan complete|dock failed|retract failed"
mp_score() {
  # the valve column: some samples spraying, not all of them; the tool never presses on the wall
  awk -F, 'NR>1 {n++; if ($6 == 1) on++; if ($5 > m) m = $5; if ($6 == 1 && $5 > mo) mo = $5} END {printf "recorded %d samples, valve open in %d, peak force %.2f N (%.2f N while spraying)\n", n, on, m, mo}' "$REC"
  ros2 run coverage_planner score_run --no-contact --samples "$REC" --wall "$REC.wall.json" --swath $SWATH \
    --spray-standoff $STANDOFF --spray-half-angle $HALF_ANGLE --min-coverage "${SPRAY_PASS_MIN_COVERAGE:-0.9}" \
    --max-peak "${SPRAY_PASS_MAX_PEAK:-0}" \
    ${SPRAY_PASS_SCORE_ARGS:-}
}

mp_init "${SPRAY_PASS_OUT:-/tmp/spray_pass}"
trap mp_stop_all EXIT

mp_start sim_launch ros2 launch sim_mujoco sim_mujoco.launch.py profile:=ur10e_telescopic controller:=jtac conditioner:=true \
  payload_mass_kg:=0.5 move_group:=true scene:=$SHARE/data/smoke_wall_scene.json ${SPRAY_PASS_LAUNCH_ARGS:-}
mp_start coverage_planner ros2 run coverage_planner coverage_planner_node

# 1. coverage plan for the wall (the wall scene schema, with walls.pcd beside it)
sleep 3
mp_coverage $SHARE/data/wall_pass_coverage_scene.json || exit $?

# 2. manager parameters (contact: false), the wall into MoveIt's planning scene as an octomap
mp_manager_yaml
mp_world_model $SHARE/data/walls.pcd

# 3. record and run
mp_record
sleep 25   # sim, controllers and move_group up
wait $WM
mp_run_and_score "$@"
exit $MP_EXIT
