#!/bin/bash
# Runs INSIDE the container of plan_playback.sh: the real planning stack on ros2_control MOCK hardware, no simulator.
#   demo.launch.py (robot_state_publisher, mock ros2_control = perfect tracking, joint_trajectory_controller, move_group)
#   reachability_node + stand_navigator (real) + mp_fake_base (cmd_vel -> odom/TF, kinematic)
#   manipulation_manager (contact:=false) executing one multi-stand ExecuteCoverage goal.
# Environment: PB_OUT (output dir, has coverage_output.json, data/walls.pcd, ur10e.rm4d), PB_MP_RVIZ (this package).
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
set -u
set +u; [ -f /ov/setup.bash ] && source /ov/setup.bash; set -u   # PB_OVERLAY: a partial install layered over the base workspace
OUT=${PB_OUT:?}; MPR=${PB_MP_RVIZ:-/mp_rviz}
PROFILE_NAME=${PB_PROFILE:-ur10e_telescopic}
PKG=$(ros2 pkg prefix sim_gazebo)
DESC=$(ros2 pkg prefix mobile_manipulator_description)/share/mobile_manipulator_description
PROFILE=$DESC/config/profiles/$PROFILE_NAME.yaml
COV=$OUT/coverage_output.json
PIDS=()
start() { local n=$1; shift; setsid "$@" > "$OUT/$n.log" 2>&1 & PIDS+=($!); }
stop_all() { for p in "${PIDS[@]}"; do kill -INT -- "-$p" 2>/dev/null; done; sleep 2; for p in "${PIDS[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done; }
trap stop_all EXIT

FULL=${PB_FULL_WALL:-1}      # 1: the whole 3 x 2 m wall (make_full_wall_fixture.py); 0: the shifted stock patch of run_multi_stand_pass.sh
if [ "$FULL" = 1 ]; then
  Z_MIN=${PB_Z_MIN:-0.5}; Z_MAX=${PB_Z_MAX:-1.9}; SPAN=${PB_MAX_SPAN:-0.5}; SWATH=${PB_SWATH:-0.2}; OVERLAP=${PB_OVERLAP:-20.0}
else
  Z_MIN=${PB_Z_MIN:-1.0}; Z_MAX=${PB_Z_MAX:-1.3}; SPAN=${PB_MAX_SPAN:-0.5}; SWATH=${PB_SWATH:-0.14}; OVERLAP=${PB_OVERLAP:-15.0}
fi
if [ ! -s "$COV" ] || [ ! -s "$OUT/ur10e.rm4d" ] || [ ! -s "$OUT/data/walls.pcd" ]; then
  DATA=$PKG/share/sim_gazebo/test/data
  if [ "$FULL" = 1 ]; then
    # the whole wall in the world (odom) frame, plane at x = PB_PLANE_X (the planner picks the arm base distance itself)
    python3 "$MPR/scripts/make_full_wall_fixture.py" "$OUT/data" --plane-x ${PB_PLANE_X:-0.98} --frame odom --step 0.01 || exit 2
    cp "$OUT/data/wall_scene.json" "$OUT/data/smoke_wall_scene.json"; SCENE=$OUT/data/wall_scene.json
  else
    DY=${PB_WALL_DY:-0.3}; WIDTH=${PB_WALL_WIDTH:-0.9}
    python3 "$PKG/lib/sim_gazebo/shift_wall_scene.py" "$DATA" "$OUT/data" 0.0 "$DY" "$WIDTH" || exit 2
    SCENE=$OUT/data/wall_pass_coverage_scene.json
  fi
  if [ ! -s "$OUT/ur10e.rm4d" ]; then
    RE=$(ros2 pkg prefix reachability)
    xacro "$DESC/urdf/mobile_manipulator.urdf.xacro" arm:=ur10e -o "$OUT/robot.urdf" || exit 2
    python3 "$RE/lib/reachability/urdf_to_chain.py" "$OUT/robot.urdf" --base arm_base_link --tip tcp_link -o "$OUT/ur10e_chain.yaml" || exit 2
    "$RE/lib/reachability/rm4d_tool" build "$OUT/ur10e_chain.yaml" "$OUT/ur10e.rm4d" --samples 3000000 --resolution 0.05 \
      --extent 2.0 --zmin -1.8 --zmax 2.0 > "$OUT/map_build.log" 2>&1 || exit 2
  fi
  rm -f "$COV"
  start coverage_planner ros2 run coverage_planner coverage_planner_node
  sleep 3
  timeout 300 ros2 run coverage_planner wall_coverage_demo_node --ros-args -p wall_scene_json:=$SCENE \
    -p output_json:=$COV -p orientation_format:=quaternion -p tool_frame_yaw_deg:=-90.0 -p swath_m:=$SWATH -p overlap_pct:=$OVERLAP -p double_pass:=${PB_DOUBLE_PASS:-false} ${PB_COV_EXTRA:-} \
    > "$OUT/coverage_demo.log" 2>&1 &
  D=$!; for _ in $(seq 600); do [ -s "$COV" ] && break; sleep 0.5; done; kill -INT $D 2>/dev/null
  [ -s "$COV" ] || { echo "no coverage plan"; exit 3; }
  kill -INT -- "-${PIDS[-1]}" 2>/dev/null   # the planner node is not needed any more
fi

start demo ros2 launch mobile_manipulator_moveit_config demo.launch.py profile:=$PROFILE_NAME rviz:=false
start fake_base python3 "$MPR/scripts/mp_fake_base.py"
start jointlog python3 "$MPR/scripts/mp_joint_logger.py" "$OUT/joints.csv" 4.0
start reachability ros2 run reachability reachability_node --ros-args -r __node:=reachability_node -p map_file:=$OUT/ur10e.rm4d -p base_profile:=$PROFILE ${PB_REACH_EXTRA:-}
start stand_navigator ros2 run stand_navigator stand_navigator_node --ros-args -p base_profile:=$PROFILE ${PB_REACH_EXTRA:-}

cat > "$OUT/manager.yaml" <<YAML
manipulation_manager:
  ros__parameters:
    run_on_start: false
    frame_id: base_footprint
    world_frame: odom
    controller_name: lift_arm_controller
    coverage_json_path: $COV
    wall_direction: [1.0, 0.0, 0.0]
    z_offset_m: 0.0
    coverage_z_step_m: 0.05
    coverage_z_min_m: $Z_MIN
    coverage_z_max_m: $Z_MAX
    coverage_eef_speed_mps: ${PB_SPEED:-0.2}
    dock_at_stroke_start: true
    serpentine_columns: ${PB_SERPENTINE:-true}
    chain_columns: ${PB_CHAIN:-true}
    dock_ik_lift_cost_weight: 2.0
    stand_max_span_m: $SPAN
    stand_arrival_command: 'ros2 run motion_planner_moveit world_model_loader_node --ros-args -p point_cloud_path:=$OUT/data/walls.pcd -p frame_id:=odom -p octomap_resolution:=0.01 >> $OUT/world_model.log 2>&1'
    wall_clear_command: 'ros2 run motion_planner_moveit world_model_loader_node --ros-args -p clear:=true >> $OUT/world_model.log 2>&1'
    dock_attempts: 6
    tcp_settle_timeout_s: 4.0
    dock_seed_joint_state: [0.1, 2.79, -2.80, -1.30, -2.20, -0.36, -1.5708]
    dock_ik_seeded: false
    dock_ik_continuity: true
    recovery_keeps_branch: true
    dock_approach_extra_m: 0.1
    dock_standoff_m: 0.04
    contact: false
    spray_extra_standoff_m: ${PB_EXTRA_STANDOFF:-0.02}
    force_topic: ""
    verify_result_path: /out/verify.json
    verify_geometric_coverage: true
    verify_swath_m: $SWATH
    base_check: "off"
    base_check_service: /reachability_node/plan_stands
    base_check_lift_max_m: 1.5
    base_check_arm_mount_z_m: 1.17
${PB_MANAGER_EXTRA:-}
YAML

sleep ${PB_WARMUP_S:-25}
# MoveIt's Cartesian segments end with a tiny nonzero velocity; the plain joint_trajectory_controller of the mock setup rejects
# such goals by default (the sims' JTAC does not). Allowing it makes the mock controller accept the plan as it is.
for _ in $(seq 20); do
  ros2 param set /lift_arm_controller allow_nonzero_velocity_at_trajectory_end true > "$OUT/controller_param.log" 2>&1 \
    && grep -q "successful" "$OUT/controller_param.log" && break
  sleep 2
done
cat "$OUT/controller_param.log"
# the wall into MoveIt's scene before the first stand (the manager reloads it at every arrival)
timeout 120 ros2 run motion_planner_moveit world_model_loader_node --ros-args -p point_cloud_path:=$OUT/data/walls.pcd \
  -p frame_id:=odom -p octomap_resolution:=0.01 > "$OUT/world_model0.log" 2>&1
start manager ros2 launch manipulation_manager manipulation_manager.launch.py hardware:=mock profile:=$PROFILE_NAME \
  use_sim_time:=false params_file:=$OUT/manager.yaml
sleep 10
timeout 3400 python3 -u "$PKG/lib/sim_gazebo/execute_coverage_client.py" --multi-stand --coverage $COV | tee "$OUT/client.log"
echo "client exit code ${PIPESTATUS[0]}"
# stay up so the user can watch the final pose and look at the logs
sleep ${PB_HOLD_S:-100000}
