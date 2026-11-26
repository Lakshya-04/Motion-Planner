#!/usr/bin/env bash
# Isaac smoke test from the host: build the robot USD if needed, start Isaac Sim in a sibling container,
# run smoke_test.py in the ROS image, stop everything. Exit status is the test's.
#
#   src/simulation/sim_isaac/scripts/run_smoke.sh [--rebuild-usd]
#
# Env: PROFILE (ur10e_telescopic), ISAAC_IMAGE (nvcr.io/nvidia/isaac-sim:5.1.0), ROS_IMAGE (motion-planner:sim-gazebo),
# ROS_DOMAIN_ID (77), ISAAC_WORK (~/.cache/mp-isaac: robot USD and Isaac caches), SIM_SECONDS (300, safety stop).
# Needs the NVIDIA container runtime. Setting ACCEPT_EULA=Y below is you accepting NVIDIA's Isaac Sim licence.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../../.." && pwd)"
PROFILE="${PROFILE:-ur10e_telescopic}"
ISAAC_IMAGE="${ISAAC_IMAGE:-nvcr.io/nvidia/isaac-sim:5.1.0}"
ROS_IMAGE="${ROS_IMAGE:-motion-planner:sim-gazebo}"
DOMAIN="${ROS_DOMAIN_ID:-77}"
WORK="${ISAAC_WORK:-${XDG_CACHE_HOME:-$HOME/.cache}/mp-isaac}"
SIM_SECONDS="${SIM_SECONDS:-300}"
NAME="mp-isaac-smoke-$$"
PROFILES="$REPO/src/description/mobile_manipulator_description/config/profiles"
USD_DIR="$WORK/assets/$PROFILE"

for img in "$ISAAC_IMAGE" "$ROS_IMAGE"; do
  docker image inspect "$img" >/dev/null 2>&1 || { echo "image $img is not here; not pulling it for you" >&2; exit 2; }
done
[ "${1:-}" = "--rebuild-usd" ] && rm -rf "$USD_DIR"

# the Isaac image runs as uid 1234, so what it writes has to go into world-writable directories
mkdir -p -m 777 "$WORK" "$WORK/assets" "$WORK/cache" "$WORK/cache/kit" "$WORK/cache/home" "$WORK/cache/data" \
  "$WORK/cache/compute" "$WORK/cache/logs"

isaac_run() {  # extra docker args..., then the python command
  docker run --gpus all --net host \
    -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=N -e OMNI_KIT_ACCEPT_EULA=YES \
    -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e ROS_DISTRO=jazzy \
    -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/sim_isaac/fastdds_udp.xml \
    -v "$WORK/cache/kit:/isaac-sim/kit/cache" -v "$WORK/cache/home:/isaac-sim/.cache" \
    -v "$WORK/cache/data:/isaac-sim/.local/share/ov/data" -v "$WORK/cache/compute:/isaac-sim/.nv/ComputeCache" \
    -v "$WORK/cache/logs:/isaac-sim/.nvidia-omniverse/logs" -v "$WORK/assets:/assets" \
    -v "$HERE:/sim_isaac/scripts:ro" -v "$PROFILES:/sim_isaac/profiles:ro" \
    -v "$REPO/docker/isaac/fastdds_udp.xml:/sim_isaac/fastdds_udp.xml:ro" \
    --entrypoint /isaac-sim/python.sh "$@"
}

if [ ! -f "$USD_DIR/robot.usd" ]; then
  echo "== robot USD for $PROFILE"
  mkdir -p -m 777 "$USD_DIR"
  docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 \
    -v "$REPO:/repo:ro" -v "$USD_DIR:/out" "$ROS_IMAGE" bash -c '
      source /opt/ros/*/setup.bash
      mkdir -p /tmp/pfx/share/ament_index/resource_index/packages
      ln -sfn /repo/src/description/mobile_manipulator_description /tmp/pfx/share/mobile_manipulator_description
      touch /tmp/pfx/share/ament_index/resource_index/packages/mobile_manipulator_description
      export AMENT_PREFIX_PATH=/tmp/pfx:$AMENT_PREFIX_PATH
      python3 /repo/src/simulation/sim_isaac/scripts/make_resolved_urdf.py /repo/src/description/mobile_manipulator_description/config/profiles/'"$PROFILE"'.yaml /out'
  isaac_run --rm "$ISAAC_IMAGE" /sim_isaac/scripts/urdf_to_usd.py --urdf "/assets/$PROFILE/robot.urdf" \
    --usd "/assets/$PROFILE/robot.usd" --profile "$PROFILE" --profile-dir /sim_isaac/profiles | grep -a -E '^(IMPORT|MISSING|JOINT lift)'
  [ -f "$USD_DIR/robot.usd" ] || { echo "USD conversion failed" >&2; exit 1; }
fi

cleanup() { docker rm -f "$NAME" >/dev/null 2>&1 || true; }
trap cleanup EXIT

echo "== starting Isaac Sim ($NAME)"
isaac_run -d --name "$NAME" "$ISAAC_IMAGE" /sim_isaac/scripts/run_isaac.py --usd "/assets/$PROFILE/robot.usd" \
  --profile "$PROFILE" --profile-dir /sim_isaac/profiles --seconds "$SIM_SECONDS" >/dev/null
for _ in $(seq 1 120); do
  docker logs "$NAME" 2>&1 | grep -aq 'start pose set' && break
  docker ps -q -f "name=$NAME" | grep -q . || { echo "Isaac container exited early:" >&2; docker logs "$NAME" 2>&1 | tail -20 >&2; exit 1; }
  sleep 2
done
docker logs "$NAME" 2>&1 | grep -aq 'start pose set' || { echo "Isaac did not get ready in 240 s" >&2; exit 1; }

echo "== smoke test ($ROS_IMAGE)"
set +e
docker run --rm --net host -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/udp.xml -e PYTHONDONTWRITEBYTECODE=1 \
  -v "$REPO:/repo:ro" -v "$REPO/docker/isaac/fastdds_udp.xml:/udp.xml:ro" "$ROS_IMAGE" bash -c \
  'source /opt/ros/*/setup.bash && python3 /repo/src/simulation/sim_isaac/scripts/smoke_test.py --profile '"$PROFILE"
rc=$?
echo "smoke_test exit code $rc"
exit $rc
