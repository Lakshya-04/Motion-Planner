#!/bin/bash
# Free-space transition comparison in the MuJoCo sim image (docs/free_space_comparison.md): starts the sim with
# move_group and the wall in the planning scene (like run_manager_pass.sh, but no manager and no motion), plans every
# transition with each MoveIt mode through free_space_compare_node, optionally with cuRobo, and writes
# $FSC_OUT/results.csv, results.md and results.png.
#
#   FSC_LOG=/path/manager.log   transitions from the ARMSTATE lines of an earlier run, or
#   FSC_TRANSITIONS=/path/t.json   a hand list (free_space_compare.py template writes one to edit)
#   FSC_OUT=/tmp/fsc            output directory
#   FSC_MODES=...               comma list (default: all MoveIt modes, see free_space_compare_node)
#   FSC_BUDGET=10               optimal-mode time budget in s;  FSC_REPEATS=1
#   FSC_CUROBO_ROBOT=/abs/lift_arm.yml [FSC_CUROBO_PRISMATIC=lift_joint] [FSC_CUROBO_WORLD=cloud.npy]
#                               also run cuRobo (needs its python env: run this in the gpu image, or run the cuRobo
#                               stage there separately, see the doc)
#   FSC_TOOLS=/repo/docker/tools  where free_space_compare.py lives (default /repo/docker/tools)
# Needs a sourced ROS environment with sim_mujoco and motion_planner_moveit installed.
set -u
PKG=$(ros2 pkg prefix sim_mujoco)
SHARE=$PKG/share/sim_mujoco
DATA=${MANAGER_PASS_DATA:-$SHARE/data}
. "$PKG/lib/sim_mujoco/manager_pass_common.sh"
TOOLS=${FSC_TOOLS:-/repo/docker/tools}
OUTDIR=${FSC_OUT:-/tmp/fsc}
mp_init "$OUTDIR/sim"
mp_profile_init "$DATA" || exit $?
DATA=$MP_DATA
trap mp_stop_all EXIT

if [ -n "${FSC_LOG:-}" ]; then
  python3 "$TOOLS/free_space_compare.py" from-log "$FSC_LOG" --out "$OUTDIR/transitions.json" || exit $?
elif [ -n "${FSC_TRANSITIONS:-}" ]; then
  cp "$FSC_TRANSITIONS" "$OUTDIR/transitions.json"
else
  echo "set FSC_LOG (a run's manager.log) or FSC_TRANSITIONS (a hand list)" >&2; exit 2
fi

mp_start sim_launch ros2 launch sim_mujoco sim_mujoco.launch.py profile:=$MP_PROFILE controller:=jtac conditioner:=true \
  payload_mass_kg:=0.5 move_group:=true scene:=$DATA/smoke_wall_scene.json ${MANAGER_PASS_LAUNCH_ARGS:-}
sleep 3
mp_world_model $DATA/${MANAGER_PASS_WORLD_PCD:-${MP_WORLD_PCD:-walls.pcd}}
sleep 25   # sim, controllers and move_group up
wait $WM

MODES=${FSC_MODES:-straight_joint,ompl_default,ompl_optimal,ompl_optimal_cart,direct,cartesian,cartesian_joint}
ros2 run motion_planner_moveit free_space_compare_node --ros-args \
  -p transitions_file:="$OUTDIR/transitions.json" -p out_file:="$OUTDIR/moveit_results.jsonl" -p modes:=$MODES \
  -p time_budget_s:=${FSC_BUDGET:-10.0} -p repeats:=${FSC_REPEATS:-1} > "$OUTDIR/compare_node.log" 2>&1 \
  || { echo "free_space_compare_node failed, see $OUTDIR/compare_node.log" >&2; exit 1; }
RESULTS=("$OUTDIR/moveit_results.jsonl")

if [ -n "${FSC_CUROBO_ROBOT:-}" ]; then
  if PYTHONPATH="${CUROBO_BRIDGE_PATH:-/repo/src/manipulation/curobo_bridge}:${PYTHONPATH:-}" \
      python3 "$TOOLS/free_space_compare_curobo.py" --moveit-results "$OUTDIR/moveit_results.jsonl" \
      --robot "$FSC_CUROBO_ROBOT" --prismatic ${FSC_CUROBO_PRISMATIC:-} ${FSC_CUROBO_WORLD:+--world-npy "$FSC_CUROBO_WORLD"} \
      --out "$OUTDIR/curobo_results.jsonl" > "$OUTDIR/curobo.log" 2>&1; then
    ros2 run motion_planner_moveit free_space_compare_node --ros-args -p fk_in:="$OUTDIR/curobo_results.jsonl" \
      -p out_file:="$OUTDIR/curobo_results_fk.jsonl" >> "$OUTDIR/compare_node.log" 2>&1 \
      && RESULTS+=("$OUTDIR/curobo_results_fk.jsonl")
  else
    echo "cuRobo stage failed (no GPU env here?), see $OUTDIR/curobo.log; the MoveIt modes are analysed anyway" >&2
  fi
fi

python3 "$TOOLS/free_space_compare.py" analyze "${RESULTS[@]}" --out-dir "$OUTDIR" --transitions "$OUTDIR/transitions.json"
echo "results: $OUTDIR/results.csv $OUTDIR/results.md $OUTDIR/results.png"
