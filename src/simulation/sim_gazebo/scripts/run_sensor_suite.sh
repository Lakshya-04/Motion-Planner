#!/bin/bash
# Start the Gazebo sim with a platform's sensor suite and check /scan and /depth_sensor/points against it.
#   SUITE_PLATFORM  platform: argument (default ranger_mini)     SUITE_PROFILE  robot profile (default ur10e_ranger_mini_telescopic)
#   SUITE_SECONDS   how long to listen (default 20)              SUITE_OUT      log directory (default /tmp/sensor_suite)
#   SUITE_DEPTH=0   lidar only (for a suite with no camera)
# Extra arguments go to the launch file, e.g. lidar_rate_hz:=20 to see an override win.
set -u
DESC=$(ros2 pkg prefix mobile_manipulator_description)
OUT=${SUITE_OUT:-/tmp/sensor_suite}
mkdir -p "$OUT"
setsid ros2 launch sim_gazebo sim.launch.py profile:=${SUITE_PROFILE:-ur10e_ranger_mini_telescopic} \
  platform:=${SUITE_PLATFORM:-ranger_mini} lidar:=true depth_sensor:=$([ "${SUITE_DEPTH:-1}" = 1 ] && echo true || echo false) "$@" > "$OUT/sim_launch.log" 2>&1 &
SIM=$!
trap 'kill -INT -- -$SIM 2>/dev/null; sleep 3; kill -KILL -- -$SIM 2>/dev/null; wait 2>/dev/null' EXIT
sleep 20
python3 "$DESC/lib/mobile_manipulator_description/sensor_suite_check.py" ${SUITE_PLATFORM:-ranger_mini} \
  --seconds ${SUITE_SECONDS:-20} $([ "${SUITE_DEPTH:-1}" = 1 ] || echo --no-depth) | tee "$OUT/check.log"
exit ${PIPESTATUS[0]}
