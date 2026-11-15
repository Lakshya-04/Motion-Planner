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
#   SPRAY_PASS_OVERLAP (percent, 20)  SPRAY_PASS_SPEED (m/s, 0.05)  SPRAY_PASS_MIN_COVERAGE (0.9)
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
MP_MANAGER_GREP="spray success|valve |Coverage plan complete|dock failed|retract failed"
mp_score() {
  # the valve column: some samples spraying, not all of them; the tool never presses on the wall
  awk -F, 'NR>1 {n++; if ($6 == 1) on++; if ($5 > m) m = $5} END {printf "recorded %d samples, valve open in %d, peak force %.2f N\n", n, on, m}' "$REC"
  ros2 run coverage_planner score_run --no-contact --samples "$REC" --wall "$REC.wall.json" --swath $SWATH \
    --spray-standoff $STANDOFF --spray-half-angle $HALF_ANGLE --min-coverage "${SPRAY_PASS_MIN_COVERAGE:-0.9}" \
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
