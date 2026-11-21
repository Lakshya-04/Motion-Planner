#!/bin/bash
# Headless scripted wall pass in the MuJoCo sim (see wall_pass_test.py): launches the sim (UR10e profile, smoke
# wall, JTAC, ft_conditioner with the generic tool payload, move_group), runs the pass, stops the sim and scores
# the recording. Extra arguments go to wall_pass_test.py. Needs a sourced ROS environment with sim_mujoco,
# ft_conditioner, compliant_trajectory_controller, coverage_planner and mobile_manipulator_moveit_config installed.
# MP_PROFILE=<name> (default ur10e_telescopic) runs another robot profile: the wall is shifted to the distance and height
# profile_reach.py derives (copy in WALL_PASS_DATA_OUT, default /tmp/wall_pass_data), the first move is a planned tcp pose
# (ready_pose.py --pose) and the strokes use that wall; see docs/sim_mujoco.md, "Running another robot profile".
set -u
PKG=$(ros2 pkg prefix sim_mujoco)
SHARE=$PKG/share/sim_mujoco
MP_PROFILE=${MP_PROFILE:-ur10e_telescopic}
export MP_PROFILE
DATA=$SHARE/data
PASS_ARGS=()
if [ "$MP_PROFILE" != ur10e_telescopic ]; then
  eval "$(python3 "$PKG/lib/sim_mujoco/profile_reach.py" --profile "$MP_PROFILE" --shell)" || exit 2
  python3 "$PKG/lib/sim_mujoco/shift_wall_scene.py" "$SHARE/data" "${WALL_PASS_DATA_OUT:-/tmp/wall_pass_data}" \
    "$MP_WALL_DX" 0 0.3 "$MP_WALL_DZ" || exit 2
  DATA=${WALL_PASS_DATA_OUT:-/tmp/wall_pass_data}
  # strokes 2 cm inside the band edges (the defaults 1.28 / 1.02 are the stock band's)
  PASS_ARGS=(--wall-x "$MP_WALL_X" --ready-pose $MP_FREE_POSE
    --z-top "$(python3 -c "print(round($MP_Z_HI - 0.02, 3))")" --z-bottom "$(python3 -c "print(round($MP_Z_LO + 0.02, 3))")")
fi
LOG=${WALL_PASS_LOG:-/tmp/sim_mujoco_wall_pass_launch.log}
REC=${WALL_PASS_RECORD:-/tmp/wall_pass.csv}
setsid ros2 launch sim_mujoco sim_mujoco.launch.py profile:=$MP_PROFILE controller:=jtac conditioner:=true \
  payload_mass_kg:=0.5 move_group:=true scene:=$DATA/smoke_wall_scene.json ${WALL_PASS_LAUNCH_ARGS:-} \
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
timeout 600 python3 "$PKG/lib/sim_mujoco/wall_pass_test.py" --record "$REC" ${PASS_ARGS[@]+"${PASS_ARGS[@]}"} "$@"
RC=$?
echo "wall_pass_test exit code $RC (launch log: $LOG)"
# The scorecard always prints the force numbers; the script fails on a failed execution or on coverage below
# WALL_PASS_MIN_COVERAGE (0.9). Add force limits with WALL_PASS_SCORE_ARGS="--max-peak 15 --max-rms 2" (and the tool pose
# gates with MANAGER_PASS_SCORE_ARGS="--max-wall-gap 0.01 --max-tilt 10 --max-bad-pose 0.05", shared with the manager pass).
SC=0
if [ -f "$REC" ]; then
  ros2 run coverage_planner score_run --samples "$REC" --wall "$REC.wall.json" --target-force "${WALL_PASS_TARGET:-10}" \
    --band 1.5 --swath 0.14 --min-coverage "${WALL_PASS_MIN_COVERAGE:-0.9}" ${MANAGER_PASS_SCORE_ARGS:-} ${WALL_PASS_SCORE_ARGS:-}
  SC=$?
  echo "score exit code $SC"
fi
[ $RC -ne 0 ] && exit $RC
exit $SC
