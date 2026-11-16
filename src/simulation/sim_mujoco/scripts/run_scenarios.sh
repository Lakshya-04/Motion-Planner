#!/bin/bash
# Runs a named list of wall scenarios through the MuJoCo manager pass and prints a table of the scores.
#
#   run_scenarios.sh [-n RUNS] [-l] [scenario ...]
#     -l        list the scenarios and exit
#     -n RUNS   runs per scenario (default 2, env SCEN_RUNS)
#     scenario  names from make_scenario_scene.py (default: plain window protrusion two_panels)
#
# Per run the scenario's wall (sim wall, planned wall, point cloud) is written to OUT/<name>_<k>/data, the pass
# runs (run_manager_pass.sh for one-stand walls, run_multi_stand_pass.sh for a wall that needs several stands), and
# the recording is scored twice with score_run:
#   cov_wall   covered fraction of the wall minus the obstacle's own footprint
#   cov_reach  covered fraction of the wall minus the obstacle grown by the planner's clearance (SCEN_CLEARANCE,
#              default 0.07 m = half the swath): the area the planner may reach
# plus peak and RMS contact force and the obstacle contacts (episodes in which the tool point was inside an
# obstacle's footprint below its top; the tool point only, not the arm's links). A run passes when the manager
# exits 0 and the scorer passes (cov_reach >= SCEN_MIN_COVERAGE, default 0.9, and no obstacle contact).
# Output in SCEN_OUT (default /tmp/scenarios): per-run folders, results.tsv, and the table on stdout.
# Needs the same ROS environment as run_manager_pass.sh / run_multi_stand_pass.sh (and their env switches apply).
set -u
PKG=$(ros2 pkg prefix sim_mujoco)
SHARE=$PKG/share/sim_mujoco
LIB=$PKG/lib/sim_mujoco
OUT=${SCEN_OUT:-/tmp/scenarios}
RUNS=${SCEN_RUNS:-2}
CLEAR=${SCEN_CLEARANCE:-0.07}
MINCOV=${SCEN_MIN_COVERAGE:-0.9}
SWATH=${SCEN_SWATH:-0.14}
TARGET=${SCEN_TARGET:-10}
DEFAULT_LIST="plain window protrusion two_panels"

while getopts "n:l" opt; do
  case $opt in
    n) RUNS=$OPTARG ;;
    l) python3 "$LIB/make_scenario_scene.py" --list; exit 0 ;;
    *) exit 2 ;;
  esac
done
shift $((OPTIND - 1))
LIST=${*:-$DEFAULT_LIST}
mkdir -p "$OUT"
TSV=$OUT/results.tsv
printf 'scenario\trun\tmanager_rc\tcov_wall\tcov_reach\tpeak_n\trms_n\tcontacts\tstatus\n' > "$TSV"

score() {   # score_run on a run; prints "cov peak rms contacts rc" ($1 = run dir, $2 = keepout margin)
  local json rc
  json=$(ros2 run coverage_planner score_run --samples "$1/run.csv" --wall "$1/run.csv.wall.json" --target-force "$TARGET" \
    --band 1.5 --swath "$SWATH" --keepout-margin "$2" --max-obstacle-contacts 0 --min-coverage "$MINCOV" 2> "$1/score.err")
  rc=$?
  echo "$json" > "$1/score_$2.json"
  python3 - "$1/score_$2.json" "$rc" <<'PY'
import json, sys
try:
    d = json.load(open(sys.argv[1]))
    print(d['coverage']['covered_fraction'], d['force']['peak_n'], d['force']['rms_error_n'], d['obstacle']['contacts'], sys.argv[2])
except Exception:
    print('nan nan nan nan', sys.argv[2])
PY
}

for name in $LIST; do
  for k in $(seq "$RUNS"); do
    O=$OUT/${name}_$k
    rm -rf "$O"; mkdir -p "$O"
    echo "=== $name run $k (logs in $O)"
    eval "$(python3 "$LIB/make_scenario_scene.py" "$SHARE/data" "$O/data" "$name")" || { echo "unknown scenario $name"; exit 2; }
    PARAMS="-p obstacle_inflation_radius_m:=$CLEAR"
    if [ "$MODE" = multi ]; then
      MULTI_STAND_OUT=$O MULTI_STAND_SCENARIO=$name MULTI_STAND_COVERAGE_EXTRA="$PARAMS" MULTI_STAND_MIN_COVERAGE=$MINCOV \
        timeout "${SCEN_TIMEOUT:-3300}" bash "$LIB/run_multi_stand_pass.sh" > "$O/pass.log" 2>&1
    else
      MANAGER_PASS_OUT=$O MANAGER_PASS_DATA=$O/data MANAGER_PASS_MIN_COVERAGE=$MINCOV \
        MP_COVERAGE_PARAMS="-p swath_m:=$SWATH -p overlap_pct:=${MANAGER_PASS_OVERLAP:-15.0} $PARAMS" \
        MANAGER_PASS_REC_ARGS="--wall-x $WALL_X --wall-y $WALL_Y0 $WALL_Y1 --scene $O/data/wall_pass_coverage_scene.json" \
        timeout "${SCEN_TIMEOUT:-1500}" bash "$LIB/run_manager_pass.sh" > "$O/pass.log" 2>&1
    fi
    RC=$?
    if [ -f "$O/run.csv" ] && [ -f "$O/run.csv.wall.json" ]; then
      read -r CW PEAK RMS CONT RCW <<< "$(score "$O" 0.0)"
      read -r CR _ _ _ RCR <<< "$(score "$O" "$CLEAR")"
    else
      CW=nan; CR=nan; PEAK=nan; RMS=nan; CONT=nan; RCR=2
    fi
    if [ "$RC" -eq 0 ] && [ "$RCR" -eq 0 ]; then STATUS=pass; else STATUS="FAIL(manager $RC, score $RCR)"; fi
    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$name" "$k" "$RC" "$CW" "$CR" "$PEAK" "$RMS" "$CONT" "$STATUS" >> "$TSV"
    tail -1 "$TSV"
    sleep 5
  done
done

python3 - "$TSV" <<'PY'
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1]), delimiter='\t'))
def f(x, p=3):
    try:
        return f'{float(x):.{p}f}'
    except ValueError:
        return x
head = ['scenario', 'run', 'cov_wall', 'cov_reach', 'peak N', 'rms N', 'contacts', 'status']
body = [[r['scenario'], r['run'], f(r['cov_wall']), f(r['cov_reach']), f(r['peak_n'], 1), f(r['rms_n'], 2), r['contacts'], r['status']]
        for r in rows]
w = [max(len(str(x)) for x in col) for col in zip(head, *body)]
for line in [head] + body:
    print('  '.join(str(x).ljust(n) for x, n in zip(line, w)))
PY
