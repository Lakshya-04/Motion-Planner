#!/bin/bash
# Headless wall pass in the MuJoCo sim driven by the real pipeline instead of scripted strokes:
#   wall scene (+ synthetic scan) -> coverage_planner (contact_wiper) -> coverage_output.json
#   -> manipulation_manager (docks, presses, strokes and retracts per column through MoveIt + JTAC)
# with the run recorded and scored like run_wall_pass.sh. Needs a sourced ROS environment with sim_mujoco,
# ft_conditioner, compliant_trajectory_controller, coverage_planner, manipulation_manager and
# mobile_manipulator_moveit_config installed. Extra arguments go to `ros2 launch manipulation_manager`.
# The wall comes from a file; run_sensed_pass.sh runs the same pass on a wall sensed by the simulated depth camera.
# MANAGER_PASS_DATA=dir takes the three wall files (smoke_wall_scene.json, wall_pass_coverage_scene.json, walls.pcd)
# from there instead of the stock smoke wall (run_scenarios.sh writes them with make_scenario_scene.py);
# MANAGER_PASS_REC_ARGS goes to record_run.py (--wall-y .. .. --scene .. for a wall that is not the stock one).
set -u
PKG=$(ros2 pkg prefix sim_mujoco)
SHARE=$PKG/share/sim_mujoco
DATA=${MANAGER_PASS_DATA:-$SHARE/data}
. "$PKG/lib/sim_mujoco/manager_pass_common.sh"
mp_init "${MANAGER_PASS_OUT:-/tmp/manager_pass}"
trap mp_stop_all EXIT

mp_start sim_launch ros2 launch sim_mujoco sim_mujoco.launch.py profile:=ur10e_telescopic controller:=jtac conditioner:=true \
  payload_mass_kg:=0.5 move_group:=true scene:=$DATA/smoke_wall_scene.json ${MANAGER_PASS_LAUNCH_ARGS:-}
# the demo rasterizes the obstacle clearance into the grid itself; the service node would add its own 0.3 m default
mp_start coverage_planner ros2 run coverage_planner coverage_planner_node --ros-args -p obstacle_inflation_radius:=0.0

# 1. coverage plan for the wall's contact region (the wall scene schema, with walls.pcd beside it)
sleep 3
mp_coverage $DATA/wall_pass_coverage_scene.json || exit $?

# 2. manager parameters: the wall is at +x, poses are authored for base_footprint at z 1.0..1.3
mp_manager_yaml

# 3. the wall into MoveIt's planning scene as an octomap (so free-space moves do not plan through it)
mp_world_model $DATA/walls.pcd

# 4. record and run
mp_record
sleep 25   # sim, controllers and move_group up
wait $WM
mp_run_and_score "$@"
exit $MP_EXIT
