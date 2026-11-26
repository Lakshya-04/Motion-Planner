#!/bin/bash
# Navigation and depth checks with the realism nodes on, in a sourced ROS environment with sim_realism, sim_mujoco
# (and sim_gazebo for REALISM_DEPTH_SIM=gazebo) installed. Runs one after another and prints a summary:
#   1. run_stand_navigation.sh with odom_realism:=$REALISM_ODOM: arrival error against the noisy /odom (what the
#      navigator sees) and against /odom_truth
#   2. run_wall_docking.sh with the same odometry: standoff and yaw error against /odom and /odom_truth
#   3. run_depth_wall.sh once clean and once with depth_realism:=$REALISM_DEPTH: wall plane fit error of both
# A check that fails stays failed: the point is to see what the controllers do with the noise.
#   REALISM_ODOM=typical  REALISM_DEPTH=typical  REALISM_SEED=1  REALISM_OUT=/tmp/realism_check
#   REALISM_DEPTH_SIM=mujoco|gazebo  REALISM_STEPS="nav dock depth"  (the steps to run)
set -u
SHARE=$(ros2 pkg prefix sim_mujoco)/lib/sim_mujoco
ODOM=${REALISM_ODOM:-typical}
DEPTH=${REALISM_DEPTH:-typical}
SEED=${REALISM_SEED:-1}
OUT=${REALISM_OUT:-/tmp/realism_check}
STEPS=${REALISM_STEPS:-nav dock depth}
DEPTH_SIM=${REALISM_DEPTH_SIM:-mujoco}
mkdir -p "$OUT"
RC=0

grep_log() { grep -hE "$1" "$2" 2>/dev/null | cut -c1-230; }

for step in $STEPS; do
  case $step in
    nav)
      echo "== stand navigation, odom_realism=$ODOM seed=$SEED"
      STAND_NAV_OUT=$OUT/nav bash "$SHARE/run_stand_navigation.sh" odom_realism:=$ODOM realism_seed:=$SEED > "$OUT/nav.console" 2>&1
      r=$?; [ $r -ne 0 ] && RC=1
      grep_log "arrival error|first stand|navigate_to_stand result|base holds|exit code" "$OUT/nav.console"
      grep_log "odom_realism drift|odom_realism:" "$OUT/nav/sim_launch.log"
      echo "-- stand navigation exit $r";;
    dock)
      echo "== wall docking, odom_realism=$ODOM seed=$SEED"
      WALL_DOCKING_OUT=$OUT/dock bash "$SHARE/run_wall_docking.sh" odom_realism:=$ODOM realism_seed:=$SEED > "$OUT/dock.console" 2>&1
      r=$?; [ $r -ne 0 ] && RC=1
      grep_log "PASS|FAIL|start:|node:|lateral|wall docking check|exit code" "$OUT/dock.console"
      grep_log "odom_realism drift" "$OUT/dock/sim.log"
      echo "-- wall docking exit $r";;
    depth)
      if [ "$DEPTH_SIM" = gazebo ]; then script=$(ros2 pkg prefix sim_gazebo)/lib/sim_gazebo/run_depth_wall.sh; else script=$SHARE/run_depth_wall.sh; fi
      for mode in off $DEPTH; do
        echo "== depth wall ($DEPTH_SIM), depth_realism=$mode seed=$SEED"
        DEPTH_WALL_NOISE=0.0 DEPTH_WALL_OUT=$OUT/depth_$mode bash "$script" depth_realism:=$mode realism_seed:=$SEED \
          > "$OUT/depth_$mode.console" 2>&1
        r=$?; [ $r -ne 0 ] && RC=1
        grep_log "PASS|FAIL|fit:|depth wall check" "$OUT/depth_$mode.console"
        echo "-- depth wall ($mode) exit $r"
      done;;
    *) echo "unknown step $step"; RC=2;;
  esac
done
echo "realism check exit code $RC (logs in $OUT)"
exit $RC
