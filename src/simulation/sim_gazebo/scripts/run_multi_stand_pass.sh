#!/bin/bash
# Closed loop in the Gazebo sim for a wall that needs SEVERAL stands (the Gazebo twin of
# sim_mujoco/scripts/run_multi_stand_pass.sh): the manager is an action server (run_on_start:=false) and one
# `execute_coverage` goal (multi_stand) does everything:
#   plan_stands for the whole plan -> columns grouped per stand -> for each stand: stow the arm, drive there
#   (navigate_to_stand, /cmd_vel and /odom of base_motion:=true), run its columns in the world (odom) frame -> next stand.
# The contact patch is MULTI_STAND_WALL_WIDTH wide (default 0.9 m, the stock patch is 0.3) starting at y = MULTI_STAND_WALL_DY
# (shift_wall_scene.py shifts the coverage scene and the cloud; the base starts at the origin). The Gazebo world wall is the
# stock 3 m x 2 m smoke wall at x = 0.915 centred on y = 0 (the world generator always centres the wall on the robot), with
# its one 5 cm obstacle moved to y -0.9..-0.6, out of the patch; the plane the coverage, the octomap and the scorer use is
# where the tcp touches it, x = 0.98 (MULTI_STAND_WALL_DX is not supported: the world wall stays put).
# Run, record in odom and score like run_manager_pass.sh.
#   MULTI_STAND_WALL_DY / _WALL_WIDTH  the patch's start y (0.3) and width (0.9) in m; _WALL_DX must stay 0 (the world wall does not move)
#   MULTI_STAND_PATCH_Z="lo,hi"    contact patch height in m (default 1.0,1.3; the tall wall is 0.6,2.0); moves the coverage scene, walls.pcd,
#                                  the manager's coverage z range and the recorder's wall band; the sim wall grows to hi + 0.3 m when that is
#                                  above the stock 2 m (shift_wall_scene.py; the tall wall is unverified in Gazebo). MULTI_STAND_SCENARIO
#                                  (make_scenario_scene.py, MuJoCo only) is not supported here: the script stops with exit 2
#   MULTI_STAND_DOUBLE_PASS=false  coverage_planner `double_pass` (default false = every strip once, the next one starts at the end the last one
#                                  ended; true = up and back down every strip, the wiper profile's own default)
#   MULTI_STAND_REACH_FRACTION=0.7       keep the tool within this fraction of MULTI_STAND_ARM_REACH (default 1.3 m, UR10e) by moving the lift (needs DOCK_IK_CONTINUITY=true; default 0 = off)
#   MULTI_STAND_ARM_REACH=1.3      the arm's reach in m for that fraction (manager `dock_ik_reach_m`)
#   MULTI_STAND_LIFT_STROKE=1.5    the lift's stroke in m (manager `dock_ik_lift_max_m`)
#   MULTI_STAND_LIFT_COST_WEIGHT=2.0     lift delta counts this many times an arm joint delta in the dock IK continuity cost (manager `dock_ik_lift_cost_weight`; 1.0 = off)
#   MULTI_STAND_MIN_CLEARANCE=0.5        stand planner floor on the chassis-to-wall clearance in m (default 0 = off)
#   MULTI_STAND_SPEED / _SWATH / _OVERLAP / _TOOL_YAW / _TARGET / _PRESS / _STANDOFF / _APPROACH_EXTRA / _DOCK_ATTEMPTS / _MIN_COVERAGE
#                                  the stroke speed, swath and overlap of the plan, tool yaw, target force, press depth, dock standoff and
#                                  approach distance, dock tries and the score's minimum coverage (defaults in the script)
#   MULTI_STAND_COVERAGE_EXTRA     more ros arguments of the coverage demo (-p obstacle_inflation_radius_m:=0.07)
#   MULTI_STAND_LAUNCH_ARGS        more arguments of the sim launch; MULTI_STAND_OUT the output directory (default /tmp/multi_stand_pass)
#   MULTI_STAND_SCORE_ARGS         extra score_run arguments
#   MULTI_STAND_DOCK_SEED='[...]'  seed of the dock IK (manager `dock_seed_joint_state`, lift first); the default is a UR10e-on-lift configuration
#   MULTI_STAND_DOCK_IK_SEEDED=true  seed the dock IK with MULTI_STAND_DOCK_SEED (manager `dock_ik_seeded`, default false here)
#   MULTI_STAND_SETTLE_TIMEOUT=90  wait_arm_settled.py timeout in s at every stand arrival; MULTI_STAND_TCP_SETTLE=4.0 the manager's
#                                  `tcp_settle_timeout_s` (s)
#   MULTI_STAND_OBSTACLES=1        keep the stock wall's obstacle in the sim (moved out of the patch; the planner never knows it)
#   MULTI_STAND_MAX_SPAN=0.5       widest stretch of wall one stand is trusted with (manager `stand_max_span_m`)
#   MULTI_STAND_DOCK_IK_CONTINUITY=true  choose the dock joint goal among IK solutions by continuity (manager `dock_ik_continuity`, default false)
#   MULTI_STAND_RECOVERY_KEEPS_BRANCH=true  recover from a short approach on the live IK branch (manager `recovery_keeps_branch`, default false)
#   MULTI_STAND_PAINT=1            live done / not done wall paint (RViz topic /mp_viz/coverage_paint; _PAINT_FEED=gazebo; default off)
#   MULTI_STAND_CANCEL_AFTER_S=N   cancel the goal N s after it was accepted instead (checks the arm stopped)
#   MULTI_STAND_MANAGER_EXTRA      extra manager.yaml lines (4-space indented)
#   STAND_MAP                      a built reachability map (skips the build, which takes a few minutes)
#   GZ_PASS_STILLS=N               colour camera on, N PNG stills into OUT/stills, one every GZ_PASS_STILLS_EVERY s
# Needs a sourced ROS environment with sim_gazebo, reachability, stand_navigator, ft_conditioner,
# compliant_trajectory_controller, ft_bridge_hardware, coverage_planner, manipulation_manager, coverage_task_msgs and
# mobile_manipulator_moveit_config. Extra arguments go to `ros2 launch manipulation_manager`.
# MANAGER_PASS_PLANNING_CONFIG=/path/planning_curobo.yaml: the manager builds its planners from that file (planning_config
# parameter, written into manager.yaml; unset = the packaged planning.yaml). Needs the cuRobo node: docs/sim_*.md.
set -u
PKG=$(ros2 pkg prefix sim_gazebo)
SHARE=$PKG/share/sim_gazebo
DATA=$SHARE/test/data
. "$PKG/lib/sim_gazebo/gz_pass_common.sh"
DESC=$(ros2 pkg prefix mobile_manipulator_description)/share/mobile_manipulator_description
PROFILE=$DESC/config/profiles/${STAND_PROFILE:-ur10e_telescopic}.yaml
DY=${MULTI_STAND_WALL_DY:-0.3}
WIDTH=${MULTI_STAND_WALL_WIDTH:-0.9}
CANCEL_AFTER=${MULTI_STAND_CANCEL_AFTER_S:-0}
MP_Z_LO=1.0; MP_Z_HI=1.3   # the contact patch's height; MULTI_STAND_PATCH_Z replaces it
GZ_PROFILE=${STAND_PROFILE:-ur10e_telescopic}

gz_init "${MULTI_STAND_OUT:-/tmp/multi_stand_pass}"
trap gz_stop_all EXIT

[ "${MULTI_STAND_SCENARIO:-}" ] && { echo "MULTI_STAND_SCENARIO (make_scenario_scene.py) is MuJoCo only; the Gazebo world wall is built from the smoke scene (see MULTI_STAND_PATCH_Z)"; exit 2; }
[ "${MULTI_STAND_WALL_DX:-0.0}" != 0.0 ] && [ "${MULTI_STAND_WALL_DX:-0.0}" != 0 ] && { echo "MULTI_STAND_WALL_DX is not supported in Gazebo: the world wall stays put"; exit 2; }
if [ -n "${MULTI_STAND_PATCH_Z:-}" ]; then
  MP_Z_LO=${MULTI_STAND_PATCH_Z%%,*}; MP_Z_HI=${MULTI_STAND_PATCH_Z##*,}
fi
python3 "$PKG/lib/sim_gazebo/shift_wall_scene.py" "$DATA" "$OUT/data" 0.0 "$DY" "$WIDTH" ${MULTI_STAND_PATCH_Z:-} || exit 2

# one line with the knobs this run used, so its output is self-describing (the same line in the MuJoCo script)
summary_line() {
  echo "multi_stand knobs: sim=gazebo profile=$GZ_PROFILE scenario=${MULTI_STAND_SCENARIO:-none} patch_z=${MP_Z_LO}..${MP_Z_HI}" \
    "double_pass=${MULTI_STAND_DOUBLE_PASS:-false} reach_fraction=${MULTI_STAND_REACH_FRACTION:-0.0} arm_reach=${MULTI_STAND_ARM_REACH:-1.3}" \
    "lift_stroke=${MULTI_STAND_LIFT_STROKE:-1.5} min_clearance=${MULTI_STAND_MIN_CLEARANCE:-0.0} dock_ik_continuity=${MULTI_STAND_DOCK_IK_CONTINUITY:-false}" \
    "max_span=${MULTI_STAND_MAX_SPAN:-0.5} speed=${MULTI_STAND_SPEED:-0.01} swath=${MULTI_STAND_SWATH:-0.14} overlap=${MULTI_STAND_OVERLAP:-15.0}" \
    "tcp_settle=${MULTI_STAND_TCP_SETTLE:-4.0} wall=0.0/${DY}/${WIDTH} paint=${MULTI_STAND_PAINT:-0} out=$OUT"
}
# the sim wall: the stock scene without its obstacle (MULTI_STAND_OBSTACLES=1 keeps it, moved out of the patch; the planner's scene never has it,
# so with it the arm drives into an obstacle the planner does not know about)
python3 - "$DATA/smoke_wall_scene.json" "$OUT/data/smoke_wall_scene.json" <<'PY' || exit 2
import json, sys
import os
scene = json.load(open(sys.argv[1]))
if os.environ.get('MULTI_STAND_OBSTACLES', '0') == '1':
    for ob in scene['obstacles']:
        ob['uv_on_wall']['u'] = [-0.9, -0.6]
else:
    scene['obstacles'] = []   # a plain wall like the MuJoCo pass (two_panels_plain); the planner's scene has no obstacle either
json.dump(scene, open(sys.argv[2], 'w'), indent=2)
PY
WALL_X=0.98
WALL_Y0=$DY
WALL_Y1=$(python3 -c "print($WIDTH + $DY)")

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

# shellcheck disable=SC2046
eval gz_start sim_launch ros2 launch sim_gazebo sim.launch.py $(gz_sim_args) base_motion:=true \
  wall_scene:=$OUT/data/smoke_wall_scene.json standoff:=0.915 ${MULTI_STAND_LAUNCH_ARGS:-}
gz_start coverage_planner ros2 run coverage_planner coverage_planner_node
gz_start reachability ros2 run reachability reachability_node --ros-args -r __node:=reachability_node -p map_file:=$MAP \
  -p base_profile:=$PROFILE -p stand_min_clearance_m:=${MULTI_STAND_MIN_CLEARANCE:-0.0}
gz_start stand_navigator ros2 run stand_navigator stand_navigator_node --ros-args -p base_profile:=$PROFILE

# 1. coverage plan for the wall's contact region, in the world frame
sleep 3
GZ_TOOL_YAW=${MULTI_STAND_TOOL_YAW:--90.0}
gz_coverage $OUT/data/wall_pass_coverage_scene.json \
  "-p swath_m:=${MULTI_STAND_SWATH:-0.14} -p overlap_pct:=${MULTI_STAND_OVERLAP:-15.0} -p double_pass:=${MULTI_STAND_DOUBLE_PASS:-false} ${MULTI_STAND_COVERAGE_EXTRA:-}" || exit $?

# 2. manager parameters: an action server, the plan stays in odom
# MANAGER_PASS_CUROBO_WORLD=1: every stand arrival also pushes the wall into the cuRobo node's world, posed from TF
# (source_frame odom -> base_link); the cuRobo node must run with --max-boxes 3000 (the wall is ~2500 voxels).
CUROBO_ARRIVAL=""
if [ -n "${MANAGER_PASS_CUROBO_WORLD:-}" ]; then
  CUROBO_ARRIVAL="; ros2 run curobo_planner curobo_world_loader --ros-args -p point_cloud_path:=$OUT/data/walls.pcd -p frame_id:=base_link -p source_frame:=odom >> $OUT/curobo_world.log 2>&1"
fi

cat > "$OUT/manager.yaml" <<YAML
manipulation_manager:
  ros__parameters:
    run_on_start: false
    frame_id: base_footprint
    world_frame: odom
    controller_name: combined_arm_lift_controller
    coverage_json_path: $COV
    wall_direction: [1.0, 0.0, 0.0]
    z_offset_m: 0.0
    coverage_z_step_m: 0.05
    coverage_z_min_m: $MP_Z_LO
    coverage_z_max_m: $MP_Z_HI
    coverage_eef_speed_mps: ${MULTI_STAND_SPEED:-0.01}
    dock_at_stroke_start: true
    stand_max_span_m: ${MULTI_STAND_MAX_SPAN:-0.5}
    stand_arrival_command: 'python3 -u $PKG/lib/sim_gazebo/wait_arm_settled.py --timeout ${MULTI_STAND_SETTLE_TIMEOUT:-90} >> $OUT/settle.log 2>&1; ros2 run motion_planner_moveit world_model_loader_node --ros-args -p point_cloud_path:=$OUT/data/walls.pcd -p frame_id:=odom -p octomap_resolution:=0.01 >> $OUT/world_model.log 2>&1${CUROBO_ARRIVAL}'
    wall_clear_command: 'ros2 run motion_planner_moveit world_model_loader_node --ros-args -p clear:=true >> $OUT/world_model.log 2>&1'
    dock_attempts: ${MULTI_STAND_DOCK_ATTEMPTS:-6}
    tcp_settle_timeout_s: ${MULTI_STAND_TCP_SETTLE:-4.0}
    dock_seed_joint_state: ${MULTI_STAND_DOCK_SEED:-[0.1, 2.79, -2.80, -1.30, -2.20, -0.36, -1.5708]}
    dock_ik_seeded: ${MULTI_STAND_DOCK_IK_SEEDED:-false}
    dock_ik_continuity: ${MULTI_STAND_DOCK_IK_CONTINUITY:-false}
    dock_ik_reach_fraction: ${MULTI_STAND_REACH_FRACTION:-0.0}
    dock_ik_reach_m: ${MULTI_STAND_ARM_REACH:-1.3}
    dock_ik_lift_joint: 0
    dock_ik_lift_min_m: 0.0
    dock_ik_lift_max_m: ${MULTI_STAND_LIFT_STROKE:-1.5}
    dock_ik_lift_cost_weight: ${MULTI_STAND_LIFT_COST_WEIGHT:-2.0}
    recovery_keeps_branch: ${MULTI_STAND_RECOVERY_KEEPS_BRANCH:-false}
    dock_approach_extra_m: ${MULTI_STAND_APPROACH_EXTRA:-0.1}
    dock_standoff_m: ${MULTI_STAND_STANDOFF:-0.04}
    contact_y_overshoot_m: ${MULTI_STAND_PRESS:-0.005}
    target_force_n: ${MULTI_STAND_TARGET:-10.0}
    base_check: "off"
    base_check_service: /reachability_node/plan_stands
    base_check_lift_max_m: 1.5
    base_check_arm_mount_z_m: 1.17
${MULTI_STAND_MANAGER_EXTRA:-}
${MANAGER_PASS_PLANNING_CONFIG:+    planning_config: $MANAGER_PASS_PLANNING_CONFIG}
${MANAGER_PASS_PLANNING_CONFIG:+    seed_start_from_live_state: true}
YAML

python3 "$PKG/lib/sim_gazebo/record_run.py" --record "$REC" --timeout 3600 --frame odom --wall-x "$WALL_X" \
  --wall-y $WALL_Y0 $WALL_Y1 --wall-z $MP_Z_LO $MP_Z_HI > "$OUT/record.log" 2>&1 &
REC_PID=$!
sleep 40   # gz, controllers and move_group up
gz_stills

# 3. the wall into MoveIt's scene (world frame; TF places it relative to base_footprint)
gz_world_model "$OUT/data/walls.pcd" odom
wait $WM

# MULTI_STAND_PAINT=1: live done / not done paint of the wall (docs/sim_overlay.md "Coverage paint"). Always on /mp_viz/coverage_paint
# (RViz: add a MarkerArray display, the mp_rviz mission_overview and layers profiles have it, off) and /mp_viz/coverage_percent;
# MULTI_STAND_PAINT_FEED=none|gazebo (draws in the gz GUI: needs the sim launched with gui:=true)
# MULTI_STAND_PAINT_ARGS adds launch arguments (swath_m:=.. frame_id:=..). Default off; a failure to start it never fails the pass.
if [ "${MULTI_STAND_PAINT:-0}" = 1 ]; then
  gz_start coverage_paint ros2 launch sim_overlay coverage_paint.launch.py wall_scene_json:=$OUT/data/wall_pass_coverage_scene.json source:=gazebo \
    frame_id:=odom plan_file:=$COV swath_m:=${MULTI_STAND_SWATH:-0.14} feed:=${MULTI_STAND_PAINT_FEED:-none} \
    mujoco_json:=$OUT/mp_sim_overlay.json ${MULTI_STAND_PAINT_ARGS:-}
fi

# 4. the manager as an action server, one goal
gz_start manager ros2 launch manipulation_manager manipulation_manager.launch.py hardware:=jtac_mock profile:=ur10e_telescopic \
  use_sim_time:=true params_file:=$OUT/manager.yaml "$@"
# the manager's TF listener needs a few seconds of /tf before the goal (a goal at once aborts: "no TF odom -> arm_base_link")
sleep 10
CLIENT_ARGS="--multi-stand --coverage $COV"
[ "$CANCEL_AFTER" != 0 ] && CLIENT_ARGS="$CLIENT_ARGS --cancel-after $CANCEL_AFTER"
timeout 3400 python3 -u "$PKG/lib/sim_gazebo/execute_coverage_client.py" $CLIENT_ARGS | tee "$OUT/client.log"
RC=${PIPESTATUS[0]}
echo "client exit code $RC (logs in $OUT)"
grep -E "stand [0-9]+:|arrived|not reached|contact success|dock failed|retract failed|stow|canceled|no stand" "$OUT/manager.log" | cut -c1-200
if [ "$CANCEL_AFTER" != 0 ]; then
  echo "admittance axes after the cancel: $(ros2 param get --spin-time 5 /combined_arm_lift_controller admittance.selected_axes 2>&1 | tr '\n' ' ')"
  summary_line
  exit $RC
fi
kill -INT "$REC_PID" 2>/dev/null; wait "$REC_PID" 2>/dev/null; REC_PID=
cat "$OUT/record.log"
SC=0
if [ -f "$REC" ]; then
  ros2 run coverage_planner score_run --samples "$REC" --wall "$REC.wall.json" --target-force "${MULTI_STAND_TARGET:-10}" \
    --band 1.5 --swath ${MULTI_STAND_SWATH:-0.14} --min-coverage "${MULTI_STAND_MIN_COVERAGE:-0.9}" ${MULTI_STAND_SCORE_ARGS:-}
  SC=$?
  echo "score exit code $SC"
fi
summary_line
[ $RC -ne 0 ] && exit $RC
exit $SC
