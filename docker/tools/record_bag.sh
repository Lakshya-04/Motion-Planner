#!/usr/bin/env bash
# Records the standard topic set of a run to an mcap bag, zstd compressed and split by size.
# Run it where ROS 2 is sourced (the sim-gazebo image, or the host), next to a running sim or stack.
#
#   docker/tools/record_bag.sh                    bag in ./bags/run_<time>
#   docker/tools/record_bag.sh -o /tmp/my_run     bag in /tmp/my_run
#   docker/tools/record_bag.sh --no-depth         leave out the depth cloud (the big one)
#   docker/tools/record_bag.sh -t 120             stop after 120 s
#   docker/tools/record_bag.sh -- /tool/valve     anything after -- is recorded too
#
# Env: BAG_MAX_MB (split size, default 512), BAG_MAX_S (split duration, default 0 = off).
# Topics that do not exist yet are fine; rosbag2 picks them up when they appear.
set -uo pipefail

OUT=""
DEPTH=1
LIMIT=""
EXTRA=()
while [ $# -gt 0 ]; do
  case "$1" in
    -o) OUT="$2"; shift ;;
    -t) LIMIT="$2"; shift ;;
    --no-depth) DEPTH=0 ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    --) shift; EXTRA=("$@"); break ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done
[ -n "$OUT" ] || OUT="bags/run_$(date +%Y%m%d_%H%M%S)"

TOPICS=(
  /joint_states
  /odom
  /scan
  /force_torque_sensor_broadcaster/wrench
  /ft_conditioner/wrench_conditioned
  /tf /tf_static
  /execute_coverage/_action/status /execute_coverage/_action/feedback
  /tool/valve
  /rosout   # cuRobo node plan telemetry lines land here, there is no telemetry topic
)
[ "$DEPTH" = 1 ] && TOPICS+=(/depth_sensor/points)

if ! command -v ros2 >/dev/null; then
  set +u; source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"; set -u
fi

ARGS=(-o "$OUT" -s mcap --storage-preset-profile zstd_fast
      --max-bag-size $(( ${BAG_MAX_MB:-512} * 1024 * 1024 )) --max-bag-duration "${BAG_MAX_S:-0}")
if [ -n "$LIMIT" ]; then
  exec timeout -s INT "$LIMIT" ros2 bag record "${ARGS[@]}" "${TOPICS[@]}" "${EXTRA[@]}"
fi
exec ros2 bag record "${ARGS[@]}" "${TOPICS[@]}" "${EXTRA[@]}"
