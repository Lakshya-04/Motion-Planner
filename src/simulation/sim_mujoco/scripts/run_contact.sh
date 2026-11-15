#!/bin/bash
# Headless contact-controlled test inside a sourced ROS environment where sim_mujoco, ft_conditioner,
# ft_bridge_hardware and compliant_trajectory_controller are installed: launches the sim (UR10e profile,
# smoke wall, JTAC, ft_conditioner), runs scripts/contact_test.py against it, stops the sim.
# Extra arguments go to contact_test.py; CONTACT_LAUNCH_ARGS adds launch arguments (e.g. force_limit_n:=11). The whole launch tree is stopped when the test returns or on timeout.
set -u
PKG=$(ros2 pkg prefix sim_mujoco)
SHARE=$PKG/share/sim_mujoco
LOG=${CONTACT_LOG:-/tmp/sim_mujoco_contact_launch.log}
# The launch runs in its own session so that the whole tree (ros2_control_node, conditioner, ...) can be
# stopped, not just the `ros2 launch` process.
setsid ros2 launch sim_mujoco sim_mujoco.launch.py profile:=ur10e_telescopic controller:=jtac conditioner:=true \
  scene:=$SHARE/data/smoke_wall_scene.json ${CONTACT_LAUNCH_ARGS:-} > "$LOG" 2>&1 &
PID=$!
STOP_WAIT=${CONTACT_STOP_WAIT:-20}      # seconds the tree gets to exit on SIGINT before SIGKILL
stop_launch() {
  kill -INT -- "-$PID" 2>/dev/null || return 0
  for _ in $(seq $((STOP_WAIT * 5))); do
    kill -0 -- "-$PID" 2>/dev/null || break
    sleep 0.2
  done
  kill -KILL -- "-$PID" 2>/dev/null
  wait "$PID" 2>/dev/null
}
trap stop_launch EXIT
timeout 400 python3 "$PKG/lib/sim_mujoco/contact_test.py" "$@"
RC=$?
echo "contact_test exit code $RC (launch log: $LOG)"
exit $RC
