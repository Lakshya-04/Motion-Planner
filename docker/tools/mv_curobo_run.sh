#!/usr/bin/env bash
# One simq job: a VISIBLE MuJoCo two-stand pass with cuRobo as the free-space planner (MoveIt/OMPL as fallback).
#   [MULTI_STAND_*=... CUROBO_OPTIMIZE=1 CUROBO_TIME_BUDGET=S CUROBO_CANDIDATES=N CUROBO_OBJECTIVE=length|duration]
#   mv_curobo_run.sh <name> <worktree>
# Steps: (a) build the manager + curobo_planner + curobo_bridge_msgs in the sim image (shared ccache);
# (b) build curobo_bridge + curobo_bridge_msgs with the fast image into $INFRA/curobo_ws/install (same flock as btf.sh);
# (c) start the cuRobo node (run_curobo_node.sh, --max-boxes 3000, same ROS_DOMAIN_ID and --ipc host as the sim) when slots says
#     "node YES"; (d) run the pass with planning_curobo_pass.yaml (a copy of planning_curobo.yaml: per-call timeout 300 s,
#     free_space_fallback moveit_ompl) and MANAGER_PASS_CUROBO_WORLD=1; (e) always stop the node and keep its log.
# Output: $INFRA/cp/<name>_out, run log cp/<name>.out, node log cp/<name>_curobo_node.log (also copied into <name>_out).
# The sim container runs as uid 1000 like the node (a root sim container cannot reach the node: docs/sim_mujoco.md).
# Env: CUROBO_WARMUP_SEC (900: how long the node may take to print "cuRobo ready"; a shared/cold GPU needed >300 s), INFRA ($HOME/mp-infra), SIM_LANE (a|b), CUROBO_ROBOT (lift_arm.yml), CUROBO_NODE_SH (run_curobo_node.sh to use).
NAME=$1; W=$2
I=${INFRA:-$HOME/mp-infra}; L=${SIM_LANE:-a}; HERE=$(cd "$(dirname "$0")" && pwd)
H=${HOSTWATCH:-$HERE/hostwatch}; [ -d "$H" ] || H=${MP_REPO:-$HOME/Motion-Planner}/docker/tools/hostwatch
NODE_SH=${CUROBO_NODE_SH:-$HERE/run_curobo_node.sh}
ROBOT=${CUROBO_ROBOT:-}
DOM=$((81 + $( [ "$L" = b ] && echo 1 || echo 0 )))
NODE=curobo-node-$NAME
OUT=$I/cp/${NAME}_out
[ -n "$NAME" ] && [ -d "$W/src" ] || { echo "usage: mv_curobo_run.sh <name> <worktree>" >&2; exit 2; }
[ -n "$ROBOT" ] || { echo "set CUROBO_ROBOT to a cuRobo robot yml made by docker/tools/generate_curobo_robot.py (e.g. with --ignore-hops 2)" >&2; exit 2; }
[ -f "$ROBOT" ] || { echo "robot config missing: $ROBOT (regenerate with docker/tools/generate_curobo_7dof.sh, a GPU job)" >&2; exit 2; }
mkdir -p "$I/cp" "$I/curobo_ws"

stop_and_collect() {
  docker logs "$NODE" > "$I/cp/${NAME}_curobo_node.log" 2>&1
  [ -d "$OUT" ] && cp "$I/cp/${NAME}_curobo_node.log" "$OUT/curobo_node.log" 2>/dev/null
  docker rm -f "$NODE" >/dev/null 2>&1
  docker rm -f "mv_sim_$L" >/dev/null 2>&1
}
trap stop_and_collect EXIT

# --- helper scripts that run inside the containers
cat > "$I/cp/build_mv_curobo.sh" <<'IN'
source /opt/ros/jazzy/setup.bash
export MAKEFLAGS=-j1 MUJOCO_ROOT=/opt/mujoco CCACHE_DIR=/ccache CCACHE_MAXSIZE=6G
cd /ws
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH; source /opt/ros/jazzy/setup.bash
rm -rf /ws/build/* /ws/install/*
colcon build --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER_LAUNCHER=/cct/bin/ccache -DCMAKE_CXX_COMPILER_LAUNCHER=/cct/bin/ccache -DCMAKE_CXX_STANDARD=17 -Doctomap_DIR=/usr/share/octomap --packages-up-to manipulation_manager sim_mujoco curobo_planner curobo_bridge_msgs 2>&1 | tail -4
echo BUILD_EXIT=${PIPESTATUS[0]}
IN
cat > "$I/cp/build_curobo_ws.sh" <<'IN'
source /opt/ros/jazzy/setup.bash
export MAKEFLAGS=-j1 CCACHE_DIR=/ccache CCACHE_MAXSIZE=6G
cd /ws
rm -rf /ws/build/* /ws/install/*
colcon --log-base /tmp/colcon_log build --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=None -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache --packages-up-to curobo_bridge curobo_bridge_msgs 2>&1 | grep -E "error|Failed|Summary" | head -20
echo BUILD_EXIT=${PIPESTATUS[0]}
IN
cat > "$I/cp/mv_in_curobo.sh" <<'IN'
# inside the sim container (uid 1000): NAME from the environment
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
: ${MULTI_STAND_SCENARIO:=two_panels_plain}; export MULTI_STAND_SCENARIO
export MULTI_STAND_OUT=/cp/${NAME}_out MULTI_STAND_FILM=1
export MULTI_STAND_LAUNCH_ARGS="out_dir:=/cp/${NAME}_mjcf viewer:=true"
ros2 service list 2>/dev/null | grep -c curobo_planner | sed 's/^/curobo services visible from the sim container: /'
bash $(ros2 pkg prefix sim_mujoco)/lib/sim_mujoco/run_multi_stand_pass.sh
echo PASS_EXIT=$?
ls /cp/${NAME}_out | head -30
IN

# --- planning config copy: timeout 300 s, free_space_fallback moveit_ompl
python3 - "$W/src/manipulation/manipulation_manager/config/planning_curobo.yaml" "$I/cp/${NAME}_planning_curobo_pass.yaml" <<'PY' || exit 2
import re, sys
t = open(sys.argv[1]).read()
t, n = re.subn(r'(^\s*timeout_sec:\s*)\d+', r'\g<1>300', t, count=1, flags=re.M)
if n != 1: sys.exit("timeout_sec not found in planning_curobo.yaml")
if not re.search(r'^\s*free_space_fallback:\s*moveit_ompl', t, re.M):
    t, n = re.subn(r'^(\s*)free_space:\s*curobo[^\n]*\n', lambda m: m.group(0) + m.group(1) + 'free_space_fallback: moveit_ompl\n', t, count=1, flags=re.M)
    if n != 1: sys.exit("free_space: curobo not found")
open(sys.argv[2], 'w').write(t)
PY

wait_slot() {  # wait_slot <class>...: all listed classes YES
  while true; do o=$(bash "$H/slots.sh"); ok=1
    for c in "$@"; do echo "$o" | grep -q "^$c  *YES" || ok=0; done
    [ $ok = 1 ] && break; sleep 30; done
}

# --- (a) sim image build
docker volume create mvl_${L}_b >/dev/null; docker volume create mvl_${L}_i >/dev/null
wait_slot build sim
docker run --rm --user root --cpuset-cpus ${SIM_CPUSET:-0-5} --memory=2800m -v $I/cctool:/cct:ro -v $I/fast/ccache:/ccache -e CCACHE_DIR=/ccache -v $W/src:/ws/src:ro -v mvl_${L}_b:/ws/build -v mvl_${L}_i:/ws/install -v $HOME/mujoco-sdk/mujoco-4.0.1:/opt/mujoco:ro -v $I/cp:/cp -w /ws motion-planner:sim bash /cp/build_mv_curobo.sh 2>&1 | tail -3
echo BUILT_SIM

# --- (b) curobo_bridge + msgs for the node, fast image, serialised with the build queue (btf.sh) on fast.lock
(
  flock 9
  wait_slot build
  mkdir -p $I/curobo_ws/build $I/curobo_ws/install
  docker run --rm --name curobo_ws_build_$NAME --user 1000:1000 -e HOME=/tmp --cpuset-cpus 0-5 --memory=2400m -v $W/src:/ws/src:ro -v $I/curobo_ws/build:/ws/build -v $I/curobo_ws/install:/ws/install -v $I/fast/ccache:/ccache -v $I/cp/build_curobo_ws.sh:/b.sh:ro --entrypoint bash motion-planner:build-fast /b.sh 2>&1 | tail -5
) 9>$I/fast.lock
echo BUILT_NODE_WS
[ -f $I/curobo_ws/install/setup.bash ] || { echo "curobo_ws install missing" >&2; exit 3; }

# --- (c) the cuRobo node: node (2.4 GB) + sim (3.7 GB) must fit together, so wait for BOTH slots and avail >= NEED_MB BEFORE
# starting the node (a node started first starves the sim slot and the whole queue); give up after GRANT_WAIT_SEC (600).
NEED_MB=${CUROBO_NEED_MB:-6200}; t0=$(date +%s)
while true; do o=$(bash "$H/slots.sh"); av=$(echo "$o" | sed -n 's/.*avail_mb=\([0-9]*\).*/\1/p' | head -1)
  if echo "$o" | grep -q '^sim  *YES' && echo "$o" | grep -q '^node  *YES' && [ "${av:-0}" -ge "$NEED_MB" ]; then break; fi
  [ $(( $(date +%s) - t0 )) -gt "${GRANT_WAIT_SEC:-600}" ] && { echo "no sim+node slot within ${GRANT_WAIT_SEC:-600} s (avail ${av:-?} MB, need $NEED_MB): giving up, node not started" >&2; exit 5; }
  sleep 20; done
export ROS_DOMAIN_ID=$DOM CUROBO_DOCKER_ARGS="--ipc host"
OPT=(); [ -n "${CUROBO_OPTIMIZE:-}" ] && OPT=(--optimize ${CUROBO_TIME_BUDGET:+--time-budget $CUROBO_TIME_BUDGET} ${CUROBO_CANDIDATES:+--candidates $CUROBO_CANDIDATES} ${CUROBO_OBJECTIVE:+--objective $CUROBO_OBJECTIVE})
bash "$NODE_SH" start --name "$NODE" --ws $I/curobo_ws --robot "$ROBOT" --max-boxes 3000 --wait-sec "${CUROBO_WARMUP_SEC:-900}" "${OPT[@]}" || { echo "node failed to start" >&2; exit 4; }

# --- (d) the visible pass (sim slot was checked together with the node above; the node's own memory now makes slots.sh say NO)
export MANAGER_PASS_PLANNING_CONFIG=/cp/${NAME}_planning_curobo_pass.yaml MANAGER_PASS_CUROBO_WORLD=1
ENVS=$(env | grep -E '^(MULTI_STAND_|MANAGER_PASS_|SC=)' | sed 's/^/-e /' | tr '\n' ' ')
docker rm -f mv_sim_$L >/dev/null 2>&1
eval docker run --rm --name mv_sim_$L --user 1000:1000 -e HOME=/tmp --network host --ipc host --gpus all --memory=3000m --memory-swap=3000m --cpuset-cpus=${SIM_CPUSET:-0-7} -e NAME=$NAME -e ROS_DOMAIN_ID=$DOM -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix $ENVS -v $W/src:/ws/src:ro -v mvl_${L}_b:/ws/build -v mvl_${L}_i:/ws/install -v $HOME/mujoco-sdk/mujoco-4.0.1:/opt/mujoco:ro -v $I/cp:/cp -w /ws motion-planner:sim bash /cp/mv_in_curobo.sh > $I/cp/$NAME.out 2>&1 &
sleep 60
while docker ps --format '{{.Names}}' | grep -q "mv_sim_$L"; do
  sleep 30
  v=$(bash $H/memwatch.sh)
  if echo "$v" | grep -q "verdict=PAUSE"; then docker pause mv_sim_$L >/dev/null; while ! bash $H/memwatch.sh | grep -q "verdict=GO"; do sleep 20; done; docker unpause mv_sim_$L >/dev/null; fi
  docker ps --format '{{.Names}}' | grep -q "^$NODE$" || echo "WARNING: cuRobo node container gone"
done
echo FINISHED
tail -25 $I/cp/$NAME.out | cut -c1-200
# (e) the EXIT trap stops the node and saves its log
