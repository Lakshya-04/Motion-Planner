#!/usr/bin/env bash
# One command for the simulator checks that run headless, no GPU and no display. The no-simulator checks are
# docker/run_all_checks.sh. Run it from the repo root on the host:
#
#   docker/run_sim_checks.sh                         the Gazebo checks
#   docker/run_sim_checks.sh --mujoco-root DIR       also the MuJoCo checks (DIR = extracted mujoco-4.x SDK)
#   docker/run_sim_checks.sh --only gazebo-smoke     one check, or a comma separated list
#   docker/run_sim_checks.sh --skip gazebo-depth-wall
#   docker/run_sim_checks.sh --no-build              reuse the build and install volumes
#   docker/run_sim_checks.sh --image IMG             image to use (default motion-planner:sim-gazebo)
#   docker/run_sim_checks.sh --out DIR               keep each check's log and a results.json in DIR
#   docker/run_sim_checks.sh --list
#
# Checks: gazebo-smoke, gazebo-depth-wall, gazebo-base-motion, mujoco-smoke (needs --mujoco-root).
# Each one starts its own simulator on its own ROS_DOMAIN_ID and stops it again. Build products live in the
# Docker volumes mp_sim_build, mp_sim_install and mp_sim_log. Exit status is non-zero if any check failed.
set -uo pipefail

IMAGE="${SIM_CHECK_IMAGE:-motion-planner:sim-gazebo}"
ONLY=""
SKIP=""
NO_BUILD=0
OUT_HOST=""
MUJOCO_ROOT_HOST=""
ALL_CHECKS=(gazebo-smoke gazebo-depth-wall gazebo-base-motion mujoco-smoke)
while [ $# -gt 0 ]; do
  case "$1" in
    --only) ONLY="$2"; shift ;;
    --skip) SKIP="$2"; shift ;;
    --no-build) NO_BUILD=1 ;;
    --image) IMAGE="$2"; shift ;;
    --out) OUT_HOST="$2"; shift ;;
    --mujoco-root) MUJOCO_ROOT_HOST="$2"; shift ;;
    --list) printf '%s\n' "${ALL_CHECKS[@]}"; exit 0 ;;
    -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [ "${INSIDE_SIM_CHECKS:-0}" != "1" ]; then
  MOUNTS=()
  if [ -n "$OUT_HOST" ]; then
    mkdir -p "$OUT_HOST" && chmod a+rwx "$OUT_HOST" || exit 2
    MOUNTS+=(-v "$(cd "$OUT_HOST" && pwd):/out" -e SIM_OUT=/out)
  fi
  if [ -n "$MUJOCO_ROOT_HOST" ]; then
    [ -f "$MUJOCO_ROOT_HOST/include/mujoco/mujoco.h" ] || { echo "no mujoco.h under $MUJOCO_ROOT_HOST/include" >&2; exit 2; }
    MOUNTS+=(-v "$(cd "$MUJOCO_ROOT_HOST" && pwd):/opt/mujoco:ro" -e MUJOCO_ROOT=/opt/mujoco)
  fi
  docker image inspect "$IMAGE" >/dev/null 2>&1 || {
    echo "image $IMAGE not found. Build it: docker build -f docker/Dockerfile --target sim-gazebo -t $IMAGE ." >&2; exit 2; }
  exec docker run --rm \
    -e INSIDE_SIM_CHECKS=1 -e SIM_ONLY="$ONLY" -e SIM_SKIP="$SKIP" -e SIM_NO_BUILD="$NO_BUILD" \
    -e ROS_DOMAIN_ID_BASE="${ROS_DOMAIN_ID_BASE:-120}" -e ROS_LOCALHOST_ONLY=1 \
    "${MOUNTS[@]}" -v "$ROOT/src:/ws/src:ro" -v "$ROOT/docker:/ws/docker:ro" \
    -v mp_sim_build:/ws/build -v mp_sim_install:/ws/install -v mp_sim_log:/ws/log \
    -w /ws "$IMAGE" bash /ws/docker/run_sim_checks.sh
fi

# ---- inside the container ----
set +u; source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"; set -u
cd /ws

HAVE_MUJOCO=0
[ -n "${MUJOCO_ROOT:-}" ] && [ -f "$MUJOCO_ROOT/include/mujoco/mujoco.h" ] && HAVE_MUJOCO=1

selected() {  # name -> 0 when it should run
  local n="$1"
  if [ -n "$SIM_ONLY" ]; then [[ ",$SIM_ONLY," == *",$n,"* ]] || return 1; fi
  [[ ",$SIM_SKIP," == *",$n,"* ]] && return 1
  return 0
}

PACKAGES=()
for n in gazebo-smoke gazebo-depth-wall gazebo-base-motion; do
  selected "$n" && PACKAGES+=(sim_gazebo wall_detector coverage_planner) && break
done
if selected mujoco-smoke && [ "$HAVE_MUJOCO" = 1 ]; then PACKAGES+=(sim_mujoco); fi

declare -A RESULT SECONDS_TAKEN
if [ "$SIM_NO_BUILD" = 0 ] && [ ${#PACKAGES[@]} -gt 0 ]; then
  echo "=== build: ${PACKAGES[*]} (and what they depend on)"
  CMAKE_ARGS=(-DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DBUILD_TESTING=OFF)
  [ -f /usr/share/octomap/octomap-config.cmake ] && CMAKE_ARGS+=(-Doctomap_DIR=/usr/share/octomap)
  colcon build --packages-up-to "${PACKAGES[@]}" --cmake-args "${CMAKE_ARGS[@]}" 2>&1 | tail -5
  [ "${PIPESTATUS[0]}" = 0 ] || { echo "build failed" >&2; echo "RESULT build=FAIL"; exit 1; }
fi
set +u; source install/setup.bash 2>/dev/null; set -u

index=0
run_check() {  # name, command...
  local name="$1"; shift
  index=$((index + 1))
  if ! selected "$name"; then RESULT[$name]=SKIP; return; fi
  echo "=== $name"
  export ROS_DOMAIN_ID=$((ROS_DOMAIN_ID_BASE + index))
  local t0=$SECONDS
  "$@" > "/tmp/$name.log" 2>&1
  local rc=$?
  SECONDS_TAKEN[$name]=$((SECONDS - t0))
  tail -4 "/tmp/$name.log" | cut -c1-160
  [ -n "${SIM_OUT:-}" ] && cp "/tmp/$name.log" "$SIM_OUT/$name.log"
  if [ $rc = 0 ]; then RESULT[$name]=PASS; else RESULT[$name]=FAIL; echo "  (full log: /tmp/$name.log, gone with the container)"; fi
}

run_check gazebo-smoke timeout 900 ros2 run sim_gazebo smoke_test.py
[ -n "${SIM_OUT:-}" ] && export DEPTH_WALL_OUT="$SIM_OUT/depth_wall"
run_check gazebo-depth-wall timeout 900 bash "$(ros2 pkg prefix sim_gazebo)/lib/sim_gazebo/run_depth_wall.sh"
run_check gazebo-base-motion timeout 900 bash "$(ros2 pkg prefix sim_gazebo)/lib/sim_gazebo/run_base_motion.sh"
if [ "$HAVE_MUJOCO" = 1 ]; then
  run_check mujoco-smoke timeout 900 bash "$(ros2 pkg prefix sim_mujoco)/lib/sim_mujoco/run_smoke.sh"
else
  RESULT[mujoco-smoke]=SKIP
  if selected mujoco-smoke && [ -n "$SIM_ONLY" ]; then
    echo "mujoco-smoke needs --mujoco-root DIR" >&2
    exit 2
  fi
fi

echo "=== summary"
fail=0
json=""
for n in "${ALL_CHECKS[@]}"; do
  r="${RESULT[$n]:-SKIP}"
  printf '  %-20s %-5s %s\n' "$n" "$r" "${SECONDS_TAKEN[$n]:+(${SECONDS_TAKEN[$n]} s)}"
  json+="${json:+, }\"$n\": {\"result\": \"$r\", \"seconds\": ${SECONDS_TAKEN[$n]:-0}}"
  [ "$r" = FAIL ] && fail=1
done
if [ -n "${SIM_OUT:-}" ]; then
  echo "{$json}" > "$SIM_OUT/results.json"
  chmod -R a+rX "$SIM_OUT"
fi
exit $fail
