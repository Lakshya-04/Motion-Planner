#!/bin/bash
# The manager pass (run_manager_pass.sh) on a SENSED wall instead of a file (ROADMAP M4 exit test): the sim runs
# with depth_sensor:=true, wall_detector turns /depth_sensor/points into wall_scene.json + walls.pcd, the scene is
# cropped to the contact region (y 0..0.3, z 1.0..1.3 in base_footprint, crop_wall_scene.py), the coverage planner
# plans that crop, world_model_loader_node loads the detected walls.pcd (the whole sensed wall, so OMPL cannot swing
# over its edge), and manipulation_manager runs; recorded and scored like run_manager_pass.sh.
# No scene data file is used for the plan or the planning scene; the sim still loads smoke_wall_scene.json as the
# physical wall. Needs wall_detector installed as well as the manager pass packages. Extra arguments go to
# `ros2 launch manipulation_manager`.
#   SENSED_PASS_OUT (default /tmp/sensed_pass)  SENSED_PASS_NOISE (sensor range noise, m, default 0.003)
#   SENSED_PASS_CROP_Y / _Z  contact crop "min max" (defaults "0 0.3" / "1.0 1.3")
# plus the MANAGER_PASS_* variables of run_manager_pass.sh.
# MP_PROFILE=<name>: another robot profile, as in run_manager_pass.sh; the wall, the crop and the camera follow it.
set -u
PKG=$(ros2 pkg prefix sim_mujoco)
SHARE=$PKG/share/sim_mujoco
. "$PKG/lib/sim_mujoco/manager_pass_common.sh"
mp_init "${SENSED_PASS_OUT:-/tmp/sensed_pass}"
mp_profile_init "$SHARE/data" || exit $?
DATA=$MP_DATA
# the camera sits at the wall's mid height (1.15 m for the stock wall)
CAM_Z=$(python3 -c "print(round(($MP_Z_LO + $MP_Z_HI) / 2, 3))")
trap mp_stop_all EXIT
rm -f "$OUT/wall_scene.json" "$OUT/walls.pcd" "$OUT/wall_scene_crop.json"

mp_start sim_launch ros2 launch sim_mujoco sim_mujoco.launch.py profile:=$MP_PROFILE controller:=jtac conditioner:=true \
  payload_mass_kg:=0.5 move_group:=true scene:=$DATA/smoke_wall_scene.json depth_sensor:=true \
  depth_sensor_pose:="0.1 0 $CAM_Z 0 0 0" depth_sensor_noise_m:=${SENSED_PASS_NOISE:-0.003} ${MANAGER_PASS_LAUNCH_ARGS:-}
SIM_T0=$SECONDS
mp_start coverage_planner ros2 run coverage_planner coverage_planner_node
mp_start wall_detector ros2 launch wall_detector wall_detector.launch.py output_dir:=$OUT target_frame:=base_footprint max_range_m:=4.0
DET=$MP_LAST

# 1. the sensed wall: wait for the detector's scene, then freeze it (stop the detector so the arm
#    entering the view cannot change the plan or the cloud mid-run)
for _ in $(seq 240); do [ -s "$OUT/wall_scene.json" ] && [ -s "$OUT/walls.pcd" ] && break; sleep 0.5; done
if [ ! -s "$OUT/wall_scene.json" ]; then echo "no wall detected (see $OUT/wall_detector.log)"; exit 4; fi
sleep 1
kill -INT -- "-$DET" 2>/dev/null
grep -E "wall: n" "$OUT/wall_detector.log" | tail -1 | cut -c1-220

# 2. coverage plan for the contact region of the sensed wall
python3 "$PKG/lib/sim_mujoco/crop_wall_scene.py" "$OUT/wall_scene.json" "$OUT/wall_scene_crop.json" \
  --y ${SENSED_PASS_CROP_Y:-0 0.3} --z ${SENSED_PASS_CROP_Z:-$MP_Z_LO $MP_Z_HI} || exit 5
mp_coverage "$OUT/wall_scene_crop.json" || exit $?

# 3. manager parameters, and the whole sensed wall into MoveIt's planning scene as an octomap
mp_manager_yaml
mp_world_model "$OUT/walls.pcd"

# 4. record and run
mp_record
while [ $((SECONDS - SIM_T0)) -lt 25 ]; do sleep 1; done   # sim, controllers and move_group up
wait $WM
mp_run_and_score "$@"
exit $MP_EXIT
