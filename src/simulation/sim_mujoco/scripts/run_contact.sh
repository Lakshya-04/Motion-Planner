#!/bin/bash
# Headless contact-controlled test inside a sourced ROS environment where sim_mujoco, ft_conditioner,
# ft_bridge_hardware and compliant_trajectory_controller are installed: launches the sim (UR10e profile,
# smoke wall, JTAC, ft_conditioner), runs scripts/contact_test.py against it, stops the sim.
# Extra arguments go to contact_test.py. The launch is killed when the test returns or on timeout.
set -u
PKG=$(ros2 pkg prefix sim_mujoco)
SHARE=$PKG/share/sim_mujoco
LOG=${CONTACT_LOG:-/tmp/sim_mujoco_contact_launch.log}
ros2 launch sim_mujoco sim_mujoco.launch.py profile:=ur10e_telescopic controller:=jtac conditioner:=true \
  scene:=$SHARE/data/smoke_wall_scene.json > "$LOG" 2>&1 &
PID=$!
trap 'kill -INT $PID 2>/dev/null; sleep 2; kill -KILL $PID 2>/dev/null' EXIT
timeout 400 python3 "$PKG/lib/sim_mujoco/contact_test.py" "$@"
RC=$?
echo "contact_test exit code $RC (launch log: $LOG)"
exit $RC
