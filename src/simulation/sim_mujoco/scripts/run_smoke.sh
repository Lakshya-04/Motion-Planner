#!/bin/bash
# Headless smoke test inside a sourced ROS environment where sim_mujoco is installed:
# launches the sim (UR10e profile + smoke wall), runs scripts/smoke_test.py against it, stops the sim.
# Uses a pid, never an unbounded loop: the launch is killed when the test returns or on timeout.
set -u
PKG=$(ros2 pkg prefix sim_mujoco)
SHARE=$PKG/share/sim_mujoco
LOG=${SMOKE_LOG:-/tmp/sim_mujoco_smoke_launch.log}
ros2 launch sim_mujoco sim_mujoco.launch.py profile:=ur10e_telescopic \
  scene:=$SHARE/data/smoke_wall_scene.json "$@" > "$LOG" 2>&1 &
PID=$!
trap 'kill -INT $PID 2>/dev/null; sleep 2; kill -KILL $PID 2>/dev/null' EXIT
timeout 240 python3 "$PKG/lib/sim_mujoco/smoke_test.py"
RC=$?
echo "smoke_test exit code $RC (launch log: $LOG)"
exit $RC
