#!/bin/bash
# Headless scripted wall pass in the MuJoCo sim (see wall_pass_test.py): launches the sim (UR10e profile, smoke
# wall, JTAC, ft_conditioner with the generic tool payload, move_group), runs the pass, stops the sim and scores
# the recording. Extra arguments go to wall_pass_test.py. Needs a sourced ROS environment with sim_mujoco,
# ft_conditioner, compliant_trajectory_controller, coverage_planner and mobile_manipulator_moveit_config installed.
set -u
PKG=$(ros2 pkg prefix sim_mujoco)
SHARE=$PKG/share/sim_mujoco
LOG=${WALL_PASS_LOG:-/tmp/sim_mujoco_wall_pass_launch.log}
REC=${WALL_PASS_RECORD:-/tmp/wall_pass.csv}
setsid ros2 launch sim_mujoco sim_mujoco.launch.py profile:=ur10e_telescopic controller:=jtac conditioner:=true \
  payload_mass_kg:=0.5 move_group:=true scene:=$SHARE/data/smoke_wall_scene.json ${WALL_PASS_LAUNCH_ARGS:-} \
  > "$LOG" 2>&1 &
PID=$!
stop_launch() {
  kill -INT -- "-$PID" 2>/dev/null || return 0
  for _ in $(seq 100); do
    kill -0 -- "-$PID" 2>/dev/null || break
    sleep 0.2
  done
  kill -KILL -- "-$PID" 2>/dev/null
  wait "$PID" 2>/dev/null
}
trap stop_launch EXIT
timeout 600 python3 "$PKG/lib/sim_mujoco/wall_pass_test.py" --record "$REC" "$@"
RC=$?
echo "wall_pass_test exit code $RC (launch log: $LOG)"
# The scorecard always prints the force numbers; the script fails on a failed execution or on coverage below
# WALL_PASS_MIN_COVERAGE (0.9). Add force limits with WALL_PASS_SCORE_ARGS="--max-peak 15 --max-rms 2".
SC=0
if [ -f "$REC" ]; then
  ros2 run coverage_planner score_run --samples "$REC" --wall "$REC.wall.json" --target-force "${WALL_PASS_TARGET:-10}" \
    --band 1.5 --swath 0.14 --min-coverage "${WALL_PASS_MIN_COVERAGE:-0.9}" ${WALL_PASS_SCORE_ARGS:-}
  SC=$?
  echo "score exit code $SC"
fi
[ $RC -ne 0 ] && exit $RC
exit $SC
