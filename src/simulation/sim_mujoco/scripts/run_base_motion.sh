#!/bin/bash
# Headless base motion test inside a sourced ROS environment where sim_mujoco is installed: launches the sim with
# base_motion:=true (UR10e profile, JTC, no wall), runs scripts/base_motion_test.py, stops the sim.
set -u
PKG=$(ros2 pkg prefix sim_mujoco)
LOG=${BASE_MOTION_LOG:-/tmp/sim_mujoco_base_motion_launch.log}
setsid ros2 launch sim_mujoco sim_mujoco.launch.py profile:=ur10e_telescopic base_motion:=true "$@" > "$LOG" 2>&1 &
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
timeout 240 python3 "$PKG/lib/sim_mujoco/base_motion_test.py"
RC=$?
echo "base_motion_test exit code $RC (launch log: $LOG)"
exit $RC
