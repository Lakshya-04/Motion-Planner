#!/usr/bin/env bash
# Starts the cuRobo planner node (curobo_bridge.ros_node) in the motion-planner:full image, detached, waits until the
# planner is warm ("cuRobo ready" in its log) and its services are visible, then returns. `stop` removes it.
#
#   docker/tools/run_curobo_node.sh start --robot /abs/path/lift_arm.yml [--ws DIR] [--max-boxes N] [--max-attempts N]
#                                         [--esdf-dims X,Y,Z [--esdf-voxel M] [--esdf-center X,Y,Z]] [--no-prismatic]
#                                         [--optimize [--time-budget S] [--candidates N] [--objective length|duration]]
#                                         [--wait-sec 300] [--name curobo-node]
#   docker/tools/run_curobo_node.sh stop|status|logs [--name curobo-node]
#
# Needs a GPU (--gpus all, about 0.7 GB of a 4 GB card) and a colcon workspace with curobo_bridge and curobo_bridge_msgs
# built (--ws DIR, default: the repo root; its install/ is mounted at /ws, the image's entrypoint sources it).
# The robot yml (docker/tools/generate_curobo_7dof.sh) stores absolute asset paths: its directory (--assets DIR, default the
# yml's directory) is mounted at the SAME path. Host network, ROS_DOMAIN_ID (default 0), RMW_IMPLEMENTATION and
# ROS_LOCALHOST_ONLY are passed through, so the manager in the sim container finds the services as /curobo_planner/*.
# The Warp kernel cache lives in ~/.cache/motion-planner-curobo (a warm cache makes a restart much faster than the first
# 40-60 s compile). Env: FULL_IMG (motion-planner:full), CUROBO_DOCKER_ARGS (extra `docker run` arguments, e.g. --ipc host).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
IMG=${FULL_IMG:-motion-planner:full}
NAME=curobo-node
WS=$REPO
ROBOT=${CUROBO_ROBOT:-}
ASSETS=
MAX_BOXES=256
ESDF_DIMS=
ESDF_VOXEL=0.04
ESDF_CENTER=0.0,0.0,0.0
PRISMATIC=1
MAX_ATTEMPTS=
OPTIMIZE=0
BUDGET=20
CANDIDATES=6
OBJECTIVE=length
WAIT_SEC=300
CACHE=${CUROBO_CACHE:-$HOME/.cache/motion-planner-curobo}
SERVICES=(plan_joint plan_pose set_world set_esdf solve_ik check_motion)

CMD=${1:-start}
[ $# -gt 0 ] && shift
while [ $# -gt 0 ]; do
  case $1 in
    --robot) ROBOT=$2; shift 2;;
    --assets) ASSETS=$2; shift 2;;
    --ws) WS=$2; shift 2;;
    --max-boxes) MAX_BOXES=$2; shift 2;;
    --esdf-dims) ESDF_DIMS=$2; shift 2;;
    --esdf-voxel) ESDF_VOXEL=$2; shift 2;;
    --esdf-center) ESDF_CENTER=$2; shift 2;;
    --max-attempts) MAX_ATTEMPTS=$2; shift 2;;
    --optimize) OPTIMIZE=1; shift;;
    --time-budget) BUDGET=$2; shift 2;;
    --candidates) CANDIDATES=$2; shift 2;;
    --objective) OBJECTIVE=$2; shift 2;;
    --no-prismatic) PRISMATIC=0; shift;;
    --wait-sec) WAIT_SEC=$2; shift 2;;
    --name) NAME=$2; shift 2;;
    -h|--help) sed -n '2,18p' "$0"; exit 0;;
    *) echo "unknown argument: $1" >&2; exit 2;;
  esac
done

in_node() {  # run a ROS command inside the container, environment sourced
  docker exec "$NAME" bash -c 'source /opt/ros/"$ROS_DISTRO"/setup.bash; [ -f /ws/install/setup.bash ] && source /ws/install/setup.bash; '"$1"
}

has_log() {  # no pipe: `grep -q` closing early would make docker logs fail under pipefail
  local logs
  logs=$(docker logs "$NAME" 2>&1)
  case $logs in *"$1"*) return 0;; esac
  return 1
}

stop_node() { docker rm -f "$NAME" >/dev/null 2>&1 || true; }

case $CMD in
  stop) stop_node; echo "stopped $NAME"; exit 0;;
  status) docker ps --filter "name=^${NAME}$" --format '{{.Names}} {{.Status}}'; docker logs --tail 5 "$NAME" 2>&1 || true; exit 0;;
  logs) exec docker logs -f "$NAME";;
  start) ;;
  *) echo "unknown command: $CMD (start|stop|status|logs)" >&2; exit 2;;
esac

[ -n "$ROBOT" ] || { echo "--robot (or CUROBO_ROBOT) is required: run docker/tools/generate_curobo_7dof.sh first" >&2; exit 2; }
[ -f "$ROBOT" ] || { echo "robot config not found: $ROBOT" >&2; exit 2; }
ROBOT=$(cd "$(dirname "$ROBOT")" && pwd)/$(basename "$ROBOT")
[ -n "$ASSETS" ] || ASSETS=$(dirname "$ROBOT")
ASSETS=$(cd "$ASSETS" && pwd)
case $ROBOT in "$ASSETS"/*) ;; *) echo "the robot yml must be inside --assets ($ASSETS)" >&2; exit 2;; esac
[ -f "$WS/install/setup.bash" ] || echo "warning: $WS/install/setup.bash missing: curobo_bridge_msgs / curobo_bridge must be built there" >&2
mkdir -p "$CACHE"

ARGS=(-p "robot:=$ROBOT" -p "max_boxes:=$MAX_BOXES")
[ "$PRISMATIC" = 1 ] && ARGS+=(-p "prismatic_joints:=[lift_joint]")
[ -n "$MAX_ATTEMPTS" ] && ARGS+=(-p "max_attempts:=$MAX_ATTEMPTS")
if [ "$OPTIMIZE" = 1 ]; then
  case $OBJECTIVE in length|duration) ;; *) echo "--objective must be length or duration" >&2; exit 2;; esac
  ARGS+=(-p "free_space_optimize:=true" -p "free_space_time_budget_s:=$BUDGET" -p "free_space_num_candidates:=$CANDIDATES"
         -p "free_space_objective:=$OBJECTIVE")
  echo "optimal free-space mode: up to $CANDIDATES candidates, budget ${BUDGET}s, objective $OBJECTIVE" \
       "(set the manager's curobo timeout_sec above the budget plus one candidate)"
fi
if [ -n "$ESDF_DIMS" ]; then
  ARGS+=(-p "esdf_dims:=[$ESDF_DIMS]" -p "esdf_voxel_size:=$ESDF_VOXEL" -p "esdf_center:=[$ESDF_CENTER]")
fi

stop_node
# shellcheck disable=SC2086  # CUROBO_DOCKER_ARGS is a word list on purpose
docker run -d --name "$NAME" --gpus all --network host --user "$(id -u):$(id -g)" \
  -e HOME=/cache -e PYTHONDONTWRITEBYTECODE=1 \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ${RMW_IMPLEMENTATION:+-e RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION"} \
  ${ROS_LOCALHOST_ONLY:+-e ROS_LOCALHOST_ONLY="$ROS_LOCALHOST_ONLY"} \
  -v "$CACHE:/cache" -v "$ASSETS:$ASSETS:ro" -v "$WS:/ws:ro" ${CUROBO_DOCKER_ARGS:-} \
  "$IMG" /opt/venv/bin/python -m curobo_bridge.ros_node --ros-args "${ARGS[@]}" >/dev/null
echo "started $NAME (domain ${ROS_DOMAIN_ID:-0}); waiting up to ${WAIT_SEC}s for the planner to warm up"

trap 'echo "interrupted: stopping $NAME" >&2; stop_node; exit 130' INT TERM
deadline=$((SECONDS + WAIT_SEC))
until has_log 'cuRobo ready'; do
  if [ "$(docker inspect -f '{{.State.Running}}' "$NAME" 2>/dev/null)" != true ]; then
    echo "node exited:" >&2; docker logs --tail 40 "$NAME" >&2; stop_node; exit 1
  fi
  if has_log 'cuRobo planner unavailable'; then
    echo "planner could not start (the node stays up but answers unavailable):" >&2
    docker logs --tail 40 "$NAME" >&2; stop_node; exit 1
  fi
  [ $SECONDS -lt $deadline ] || { echo "no 'cuRobo ready' after ${WAIT_SEC}s" >&2; docker logs --tail 40 "$NAME" >&2; stop_node; exit 1; }
  sleep 3
done
docker logs "$NAME" 2>&1 | grep 'cuRobo ready'

while :; do
  listed=$(in_node 'ros2 service list' 2>/dev/null || true)
  missing=
  for s in "${SERVICES[@]}"; do
    case $listed in *"/curobo_planner/$s"*) ;; *) missing="$missing $s";; esac
  done
  [ -z "$missing" ] && break
  [ $SECONDS -lt $deadline ] || { echo "services missing after ${WAIT_SEC}s:$missing" >&2; stop_node; exit 1; }
  sleep 2
done
trap - INT TERM
echo "services up: ${SERVICES[*]} under /curobo_planner (stop with: $0 stop --name $NAME)"
