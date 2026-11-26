#!/bin/bash
# Closed loop in the MuJoCo sim: plan stands for the wall, drive the base to the first one, run the manager pass FROM
# THERE in the world (odom) frame, record and score it.
#   wall scene moved by (STAND_PASS_WALL_DX, STAND_PASS_WALL_DY) so the first stand is far from the base's start
#   -> coverage_planner (world frame poses) -> reachability plan_stands -> stand_navigator drives to stand 0
#   -> manipulation_manager with world_frame:=odom (each column transformed into base_footprint from TF)
# Needs a sourced ROS environment with sim_mujoco, reachability, stand_navigator, ft_conditioner,
# compliant_trajectory_controller, coverage_planner, manipulation_manager and mobile_manipulator_moveit_config.
# Extra arguments go to `ros2 launch manipulation_manager`.
set -u
PKG=$(ros2 pkg prefix sim_mujoco)
SHARE=$PKG/share/sim_mujoco
DESC=$(ros2 pkg prefix mobile_manipulator_description)/share/mobile_manipulator_description
PROFILE=$DESC/config/profiles/${STAND_PROFILE:-ur10e_telescopic}.yaml
OUT=${STAND_PASS_OUT:-/tmp/stand_pass}
DX=${STAND_PASS_WALL_DX:-0.0}
DY=${STAND_PASS_WALL_DY:-0.9}
mkdir -p "$OUT"
COV=$OUT/coverage_output.json
REC=$OUT/run.csv

# the wall at x = 0.98 + DX, y 0..0.3 + DY (world frame; the base starts at the origin facing +x)
python3 "$PKG/lib/sim_mujoco/shift_wall_scene.py" "$SHARE/data" "$OUT/data" "$DX" "$DY" || exit 2
WALL_X=$(python3 -c "print(0.98 + $DX)")
WALL_Y0=$(python3 -c "print($DY)")
WALL_Y1=$(python3 -c "print(0.3 + $DY)")

MAP=${STAND_MAP:-}
if [ -z "$MAP" ]; then
  MAP=$OUT/ur10e.rm4d
  RE=$(ros2 pkg prefix reachability)
  xacro "$DESC/urdf/mobile_manipulator.urdf.xacro" arm:=ur10e -o "$OUT/robot.urdf" || exit 2
  python3 "$RE/lib/reachability/urdf_to_chain.py" "$OUT/robot.urdf" --base arm_base_link --tip tcp_link \
    -o "$OUT/ur10e_chain.yaml" || exit 2
  "$RE/lib/reachability/rm4d_tool" build "$OUT/ur10e_chain.yaml" "$MAP" --samples 3000000 --resolution 0.05 \
    --extent 2.0 --zmin -1.8 --zmax 2.0 > "$OUT/map_build.log" 2>&1 || { cat "$OUT/map_build.log"; exit 2; }
fi

setsid ros2 launch sim_mujoco sim_mujoco.launch.py profile:=${STAND_PROFILE:-ur10e_telescopic} controller:=jtac \
  conditioner:=true payload_mass_kg:=0.5 move_group:=true base_motion:=true scene:=$OUT/data/smoke_wall_scene.json \
  ${STAND_PASS_LAUNCH_ARGS:-} > "$OUT/sim_launch.log" 2>&1 &
SIM=$!
setsid ros2 run coverage_planner coverage_planner_node > "$OUT/coverage_planner.log" 2>&1 &
CP=$!
setsid ros2 run reachability reachability_node --ros-args -r __node:=reachability_node -p map_file:=$MAP \
  -p base_profile:=$PROFILE > "$OUT/reachability.log" 2>&1 &
RN=$!
setsid ros2 run stand_navigator stand_navigator_node --ros-args -p base_profile:=$PROFILE \
  > "$OUT/stand_navigator.log" 2>&1 &
SN=$!
stop_all() {
  [ -n "${REC_PID:-}" ] && kill -INT "$REC_PID" 2>/dev/null && wait "$REC_PID" 2>/dev/null
  for p in $SN $RN $CP $SIM; do kill -INT -- "-$p" 2>/dev/null; done
  sleep 3
  for p in $SN $RN $CP $SIM; do kill -KILL -- "-$p" 2>/dev/null; done
  wait 2>/dev/null
}
trap stop_all EXIT

# 1. coverage plan for the wall's contact region, in the world frame
rm -f "$COV"
sleep 3
timeout 120 ros2 run coverage_planner wall_coverage_demo_node --ros-args -p double_pass:=${STAND_PASS_DOUBLE_PASS:-false} \
  -p wall_scene_json:=$OUT/data/wall_pass_coverage_scene.json -p output_json:=$COV \
  -p orientation_format:=quaternion -p tool_frame_yaw_deg:=${STAND_PASS_TOOL_YAW:--90.0} \
  -p swath_m:=${STAND_PASS_SWATH:-0.14} -p overlap_pct:=${STAND_PASS_OVERLAP:-15.0} > "$OUT/coverage_demo.log" 2>&1 &
DEMO=$!
for _ in $(seq 100); do [ -s "$COV" ] && break; sleep 0.5; done
kill -INT $DEMO 2>/dev/null
if [ ! -s "$COV" ]; then echo "no coverage plan produced (see $OUT/coverage_demo.log)"; exit 3; fi

# 2. manager parameters: the plan stays in odom, the manager transforms each column into base_footprint
cat > "$OUT/manager.yaml" <<YAML
manipulation_manager:
  ros__parameters:
    frame_id: base_footprint
    world_frame: odom
    coverage_json_path: $COV
    wall_direction: [1.0, 0.0, 0.0]
    z_offset_m: 0.0
    coverage_z_step_m: 0.05
    coverage_z_min_m: 1.0
    coverage_z_max_m: 1.3
    coverage_eef_speed_mps: ${STAND_PASS_SPEED:-0.01}
    dock_at_stroke_start: true
    dock_attempts: ${STAND_PASS_DOCK_ATTEMPTS:-6}
    dock_standoff_m: ${STAND_PASS_STANDOFF:-0.04}
    contact_y_overshoot_m: ${STAND_PASS_PRESS:-0.0015}
    target_force_n: ${STAND_PASS_TARGET:-10.0}
    base_check: ${STAND_PASS_BASE_CHECK:-warn}
    base_check_service: /reachability_node/plan_stands
    base_check_lift_max_m: 1.5
    base_check_arm_mount_z_m: 1.17
YAML

# 3. record in the world frame (the wall is where the scene put it)
python3 "$PKG/lib/sim_mujoco/record_run.py" --record "$REC" --timeout 1800 --frame odom --wall-x "$WALL_X" \
  --wall-y $WALL_Y0 $WALL_Y1 --wall-z 1.0 1.3 > "$OUT/record.log" 2>&1 &
REC_PID=$!
sleep 25   # sim, controllers and move_group up

# 4. plan stands, drive to the first one
timeout 300 python3 -u "$PKG/lib/sim_mujoco/stand_drive.py" --coverage "$COV" --profile "$PROFILE" --wall-x "$WALL_X" \
  --index 0 | tee "$OUT/stand_drive.log"
DRC=${PIPESTATUS[0]}
if [ "$DRC" -ne 0 ]; then echo "stand drive failed ($DRC)"; exit 4; fi

# 5. the wall into MoveIt's scene (world frame, after the drive: TF places it relative to base_footprint)
timeout 60 ros2 run motion_planner_moveit world_model_loader_node --ros-args \
  -p point_cloud_path:=$OUT/data/walls.pcd -p frame_id:=odom -p octomap_resolution:=0.01 \
  > "$OUT/world_model.log" 2>&1
python3 "$PKG/lib/sim_mujoco/ready_pose.py" | tee "$OUT/ready_pose.log"

# 6. the manager pass from the stand
timeout 1200 ros2 launch manipulation_manager manipulation_manager.launch.py hardware:=jtac_mock use_sim_time:=false \
  params_file:=$OUT/manager.yaml "$@" > "$OUT/manager.log" 2>&1
RC=$?
echo "manager exit code $RC (logs in $OUT)"
grep -E "base stance|base at|contact success|Coverage plan complete|dock failed|retract failed" "$OUT/manager.log" | cut -c1-220
kill -INT "$REC_PID" 2>/dev/null; wait "$REC_PID" 2>/dev/null; REC_PID=
cat "$OUT/record.log"
SC=0
if [ -f "$REC" ]; then
  ros2 run coverage_planner score_run --samples "$REC" --wall "$REC.wall.json" --target-force "${STAND_PASS_TARGET:-10}" \
    --band 1.5 --swath ${STAND_PASS_SWATH:-0.14} --min-coverage "${STAND_PASS_MIN_COVERAGE:-0.9}" ${STAND_PASS_SCORE_ARGS:-}
  SC=$?
  echo "score exit code $SC"
fi
[ $RC -ne 0 ] && exit $RC
exit $SC
