#!/usr/bin/env bash
# One command for every check that needs no simulator, GPU or hardware (CI runs part of this on free
# runners; see CONTRIBUTING.md and docs/docker.md#run-all-checks). Run it from the repo root on the host:
#
#   docker/run_all_checks.sh                  everything below
#   docker/run_all_checks.sh --skip-sims      leave out sim_gazebo, sim_mujoco, sim_isaac (packages and Python tests)
#   docker/run_all_checks.sh --no-build       reuse the build and install volumes, only test
#   docker/run_all_checks.sh --image IMG      image to use (default motion-planner:build)
#   docker/run_all_checks.sh --workers N      parallel jobs for colcon and make (default 2)
#
# Steps: (1) colcon build and colcon test of all packages, (2) pytest over the Python tests under
# src/ and docker/tools/tests (both in the container), (3) docker/tools/check_public_readiness.py (host). The simulator scripts
# (run_smoke.sh, contact_control_test.py, ...) are not part of this: they start a simulation.
# Build products live in the Docker volumes mp_build, mp_install and mp_log, not in the checkout.
# Exit status is non-zero when any step failed; a summary is printed at the end.
set -uo pipefail

IMAGE="${CHECK_IMAGE:-motion-planner:build}"
WORKERS="${WORKERS:-2}"
SKIP_SIMS="${SKIP_SIMS:-0}"
NO_BUILD="${NO_BUILD:-0}"
while [ $# -gt 0 ]; do
  case "$1" in
    --skip-sims) SKIP_SIMS=1 ;;
    --no-build) NO_BUILD=1 ;;
    --image) IMAGE="$2"; shift ;;
    --workers) WORKERS="$2"; shift ;;
    -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [ "${INSIDE_CHECKS:-0}" != "1" ]; then
  # Host side: steps 1 and 2 run inside the image (sources mounted read-only), step 3 runs here
  # because it reads `git ls-files`.
  LOG="$(mktemp)"
  docker run --rm \
    -e INSIDE_CHECKS=1 -e SKIP_SIMS="$SKIP_SIMS" -e NO_BUILD="$NO_BUILD" -e WORKERS="$WORKERS" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}" \
    -v "$ROOT/src:/ws/src:ro" -v "$ROOT/docker:/ws/docker:ro" \
    -v mp_build:/ws/build -v mp_install:/ws/install -v mp_log:/ws/log \
    -w /ws "$IMAGE" bash /ws/docker/run_all_checks.sh 2>&1 | tee "$LOG"
  echo "=== 3/3 public-readiness scan (host) ==="
  python3 docker/tools/check_public_readiness.py; rr=$?
  fail=$rr
  echo "=== summary ==="
  for k in colcon pytest; do
    v="$(sed -n "s/^RESULT $k=//p" "$LOG" | tail -1)"
    if [ "$v" = 0 ]; then echo "  $k: ok"; else echo "  $k: FAILED (${v:-no result})"; fail=1; fi
  done
  if [ $rr = 0 ]; then echo "  readiness: ok"; else echo "  readiness: FAILED"; fi
  grep -E "^(Summary:|pytest summary:|[0-9]+ (passed|failed))" "$LOG" | tail -5
  exit $fail
fi

# ---- inside the container ----
set +u; source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"; set -u
export MAKEFLAGS="-j${WORKERS}" CMAKE_BUILD_PARALLEL_LEVEL="${WORKERS}"
declare -A RESULT
cd /ws
SKIP_ARGS=()
PYTEST_DIRS=(docker/tools/tests)
[ "$SKIP_SIMS" = 1 ] && SKIP_ARGS=(--packages-skip sim_gazebo sim_mujoco sim_isaac)
while IFS= read -r d; do
  if [ "$SKIP_SIMS" = 1 ] && [[ "$d" == src/simulation/* ]]; then continue; fi
  PYTEST_DIRS+=("$d")
done < <(find src -type f \( -name 'test_*.py' -o -name '*_test.py' \) -path '*/test/*' -printf '%h\n' | sort -u)

CMAKE_ARGS=(-DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17)
# Same octomap pin as docker/build.sh (two octomap versions in one process corrupt the heap at exit).
[ -f /usr/share/octomap/octomap-config.cmake ] && CMAKE_ARGS+=(-Doctomap_DIR=/usr/share/octomap)

echo "=== 1/3 colcon build and test (workers=$WORKERS skip_sims=$SKIP_SIMS) ==="
rc=0
if [ "$NO_BUILD" = 0 ]; then
  colcon build --parallel-workers "$WORKERS" "${SKIP_ARGS[@]}" --cmake-args "${CMAKE_ARGS[@]}" || rc=1
fi
if [ $rc = 0 ]; then
  colcon test --parallel-workers "$WORKERS" "${SKIP_ARGS[@]}" --event-handlers console_direct- || true
  colcon test-result --verbose | tail -40
  colcon test-result | tail -1 | sed "s/^/Summary: /"
  colcon test-result >/dev/null 2>&1 || rc=1
fi
RESULT[colcon]=$rc

echo "=== 2/3 pytest (${PYTEST_DIRS[*]}) ==="
set +u; source install/setup.bash 2>/dev/null; set -u
# One pytest run per directory: a module-level skip or import error in one directory (no torch,
# no MuJoCo) must not hide the others, and same-named test files in different packages collide.
py_rc=0
for d in "${PYTEST_DIRS[@]}"; do
  out="$(python3 -m pytest -q -p no:cacheprovider --import-mode=importlib "$d" 2>&1)"; r=$?
  [ $r = 5 ] && r=0   # nothing collected (everything skipped at import): not a failure
  [ $r != 0 ] && { echo "$out" | tail -25; py_rc=1; }
  echo "  $d: $(echo "$out" | tail -1)"
done
RESULT[pytest]=$py_rc
echo "RESULT colcon=${RESULT[colcon]}"
echo "RESULT pytest=${RESULT[pytest]}"
