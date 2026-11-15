#!/bin/bash
# Headless wall pass in the MuJoCo sim driven by the real pipeline instead of scripted strokes:
#   wall scene (+ synthetic scan) -> coverage_planner (contact_wiper) -> coverage_output.json
#   -> manipulation_manager (docks, presses, strokes and retracts per column through MoveIt + JTAC)
# with the run recorded and scored like run_wall_pass.sh. Needs a sourced ROS environment with sim_mujoco,
# ft_conditioner, compliant_trajectory_controller, coverage_planner, manipulation_manager and
# mobile_manipulator_moveit_config installed. Extra arguments go to `ros2 launch manipulation_manager`.
set -u
PKG=$(ros2 pkg prefix sim_mujoco)
SHARE=$PKG/share/sim_mujoco
OUT=${MANAGER_PASS_OUT:-/tmp/manager_pass}
mkdir -p "$OUT"
LOG=$OUT/sim_launch.log
REC=$OUT/run.csv
COV=$OUT/coverage_output.json

setsid ros2 launch sim_mujoco sim_mujoco.launch.py profile:=ur10e_telescopic controller:=jtac conditioner:=true \
  payload_mass_kg:=0.5 move_group:=true scene:=$SHARE/data/smoke_wall_scene.json ${MANAGER_PASS_LAUNCH_ARGS:-} \
  > "$LOG" 2>&1 &
SIM=$!
setsid ros2 run coverage_planner coverage_planner_node > "$OUT/coverage_planner.log" 2>&1 &
CP=$!
stop_all() {
  [ -n "${REC_PID:-}" ] && kill -INT "$REC_PID" 2>/dev/null && wait "$REC_PID" 2>/dev/null
  for p in $CP $SIM; do kill -INT -- "-$p" 2>/dev/null; done
  sleep 3
  for p in $CP $SIM; do kill -KILL -- "-$p" 2>/dev/null; done
  wait 2>/dev/null
}
trap stop_all EXIT

# 1. coverage plan for the wall's contact region (the wall scene schema, with walls.pcd beside it)
rm -f "$COV"
sleep 3
timeout 120 ros2 run coverage_planner wall_coverage_demo_node --ros-args \
  -p wall_scene_json:=$SHARE/data/wall_pass_coverage_scene.json -p output_json:=$COV \
  -p orientation_format:=quaternion -p tool_frame_yaw_deg:=${MANAGER_PASS_TOOL_YAW:--90.0} \
  -p swath_m:=${MANAGER_PASS_SWATH:-0.14} -p overlap_pct:=${MANAGER_PASS_OVERLAP:-15.0} > "$OUT/coverage_demo.log" 2>&1 &
DEMO=$!
for _ in $(seq 100); do [ -s "$COV" ] && break; sleep 0.5; done
kill -INT $DEMO 2>/dev/null
if [ ! -s "$COV" ]; then echo "no coverage plan produced (see $OUT/coverage_demo.log)"; exit 3; fi

# 2. manager parameters: the wall is at +x, poses are authored for base_footprint at z 1.0..1.3
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
YAML

# 3. the wall into MoveIt's planning scene as an octomap (so free-space moves do not plan through it)
timeout 60 ros2 run motion_planner_moveit world_model_loader_node --ros-args \
  -p point_cloud_path:=$SHARE/data/walls.pcd -p frame_id:=base_footprint -p octomap_resolution:=0.01 \
  > "$OUT/world_model.log" 2>&1 &
WM=$!

# 4. record and run
python3 "$PKG/lib/sim_mujoco/record_run.py" --record "$REC" --timeout 1200 > "$OUT/record.log" 2>&1 &
REC_PID=$!
sleep 25   # sim, controllers and move_group up
wait $WM
python3 "$PKG/lib/sim_mujoco/ready_pose.py" | tee "$OUT/ready_pose.log"
timeout 900 ros2 launch manipulation_manager manipulation_manager.launch.py hardware:=jtac_mock use_sim_time:=false \
  params_file:=$OUT/manager.yaml "$@" > "$OUT/manager.log" 2>&1
RC=$?
echo "manager exit code $RC (logs in $OUT)"
grep -E "contact success|Coverage plan complete|dock failed|retract failed" "$OUT/manager.log" | cut -c1-200
kill -INT "$REC_PID" 2>/dev/null; wait "$REC_PID" 2>/dev/null; REC_PID=
cat "$OUT/record.log"
SC=0
if [ -f "$REC" ]; then
  ros2 run coverage_planner score_run --samples "$REC" --wall "$REC.wall.json" --target-force "${MANAGER_PASS_TARGET:-10}" \
    --band 1.5 --swath ${MANAGER_PASS_SWATH:-0.14} --min-coverage "${MANAGER_PASS_MIN_COVERAGE:-0.9}" ${MANAGER_PASS_SCORE_ARGS:-}
  SC=$?
  echo "score exit code $SC"
fi
[ $RC -ne 0 ] && exit $RC
exit $SC
