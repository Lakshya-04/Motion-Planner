#!/usr/bin/env bash
# Run the pytest suites of the ROS-free Python packages and the repository tools on the host (no ROS, no docker).
#
#   docker/tools/run_host_python_tests.sh
#
# Needs: python3 -m pip install numpy pyyaml pytest matplotlib xacro
# Each suite runs in its own pytest process (test module names repeat across packages). A package that is not in
# the tree is reported as SKIPPED, so the list can name packages that land later. Exit status is non-zero if any
# suite fails.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT" || exit 2

# package directory name | test directory under it (a package may sit in any src/<group>/)
SUITES=(
  "stand_navigator|test"
  "sim_overlay|test"
  "scenario_sync|test"
  "standoff_advisor|test"
  "wall_detector|test/python"
  "mp_rviz|test"
)
declare -a NAMES RESULTS
fail=0
run_suite() {  # name dir
  local name="$1" dir="$2"
  python3 -m pytest "$dir" -q -p no:cacheprovider; rc=$?
  # pytest exit 5 = no tests collected (a package whose tests are C++ only): not a failure
  if [ $rc -eq 0 ]; then RESULTS+=("PASS"); elif [ $rc -eq 5 ]; then RESULTS+=("NO-TESTS"); else RESULTS+=("FAIL"); fail=1; fi
  NAMES+=("$name")
}
for s in "${SUITES[@]}"; do
  pkg="${s%%|*}"; sub="${s#*|}"
  pkgdir="$(find src -maxdepth 3 -type d -name "$pkg" -not -path '*/build/*' | head -n 1)"
  if [[ -z "$pkgdir" || ! -d "$pkgdir/$sub" ]]; then NAMES+=("$pkg"); RESULTS+=("SKIPPED (not in tree)"); continue; fi
  echo "=== $pkg ($pkgdir/$sub)"
  # PYTHONPATH: the package itself (src/<group>/<pkg>) so `import <pkg>` works without colcon
  PYTHONPATH="$ROOT/$pkgdir:${PYTHONPATH:-}" run_suite "$pkg" "$pkgdir/$sub"
done
echo "=== docker/tools/tests"
run_suite "docker/tools/tests" docker/tools/tests

printf '\n%-28s %s\n' SUITE RESULT
for i in "${!NAMES[@]}"; do printf '%-28s %s\n' "${NAMES[$i]}" "${RESULTS[$i]}"; done
exit "$fail"
