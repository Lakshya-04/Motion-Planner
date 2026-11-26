#!/bin/bash
# Gazebo run: does the EKF track the truth better than raw odometry? In a sourced ROS environment with sim_gazebo,
# sim_realism and base_localisation installed. Starts the sim in the room world with typical odometry noise, the IMU and
# lidar, the full EKF (odom + imu + scan matching, owns the TF) and two side filters on the same noisy /odom for the
# comparison (odom only, odom + imu), then drives two laps of a 2 m square and prints the errors against /odom_truth.
#   LOC_OUT=/tmp/localisation_check  LOC_SEED=1  LOC_ODOM=typical  LOC_PLATFORM=omni  LOC_LAPS=2
#   LOC_IMU_ARGS="imu_gyro_noise:=0.002 imu_gyro_bias:=0.001"   extra sim arguments, e.g. for a noisy IMU
set -u
OUT=${LOC_OUT:-/tmp/localisation_check}
SEED=${LOC_SEED:-1}
ODOM=${LOC_ODOM:-typical}
PLATFORM=${LOC_PLATFORM:-omni}
mkdir -p "$OUT"
WORLD=$(ros2 pkg prefix sim_gazebo)/share/sim_gazebo/worlds/room.sdf
LIB=$(ros2 pkg prefix base_localisation)/lib/base_localisation

PIDS=()
start() { local n=$1; shift; setsid "$@" > "$OUT/$n.log" 2>&1 & PIDS+=($!); }
stop_all() {
  for p in "${PIDS[@]}"; do kill -INT -- "-$p" 2>/dev/null; done
  sleep 3
  for p in "${PIDS[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done
  pkill -KILL -f "gz sim" 2>/dev/null
  wait 2>/dev/null
}
trap stop_all EXIT

# shellcheck disable=SC2086
start sim ros2 launch sim_gazebo sim.launch.py profile:=ur10e_telescopic base_motion:=true world:=$WORLD \
  odom_realism:=$ODOM realism_seed:=$SEED imu:=true lidar:=true localisation:=true localisation_scan_match:=true \
  localisation_platform:=$PLATFORM ${LOC_IMU_ARGS:-}
start ekf_odom ros2 launch base_localisation localisation.launch.py name:=ekf_odom platform:=$PLATFORM imu:=false \
  publish_tf:=false output_topic:=/ekf/odom_only use_sim_time:=true
start ekf_imu ros2 launch base_localisation localisation.launch.py name:=ekf_imu platform:=$PLATFORM imu:=true \
  publish_tf:=false output_topic:=/ekf/odom_imu use_sim_time:=true

timeout 600 python3 "$LIB/localisation_check.py" --laps "${LOC_LAPS:-2}" --csv "$OUT/tracks.csv" \
  --track raw=/odom --track ekf_odom=/ekf/odom_only --track ekf_imu=/ekf/odom_imu --track ekf_full=/odometry/filtered \
  | tee "$OUT/result.txt"
RC=${PIPESTATUS[0]}
grep -hE "odom_realism drift|odom_realism:" "$OUT/sim.log" | cut -c1-230
grep -h "scan_match:" "$OUT/sim.log" | tail -1
echo "localisation check exit code $RC (logs in $OUT)"
exit $RC
