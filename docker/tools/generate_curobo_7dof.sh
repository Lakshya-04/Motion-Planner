#!/usr/bin/env bash
# Generates the 7-joint cuRobo robot config (lift_joint, arm_shoulder_pan_joint .. arm_wrist_3_joint) that the
# manager's `lift_arm` group needs, from a robot profile. Same steps as docs/curobo_v2_notes.md ("7-joint robot")
# and docs/robot_assets.md, in one script:
#   1. robot.urdf, full model with the moving lift     (generate_robot_assets.sh --only urdf, motion-planner:build)
#   2. package:// meshes resolved, primitives to STL   (resolve_package_uris.py, primitives_to_meshes.py, motion-planner:build)
#   3. the cuRobo config with fitted spheres           (generate_curobo_robot.py, motion-planner:gpu, needs the GPU)
#
#   docker/tools/generate_curobo_7dof.sh [--profile P.yaml] [--out DIR] [--tool-frame tcp_link] [--skip-urdf]
#
# Output: DIR/lift_arm.yml (default DIR build/robot_assets/ur10e_telescopic). The yml stores ABSOLUTE paths under DIR,
# so the node container must mount DIR at the same path (run_curobo_node.sh does).
# Env: BUILD_IMG (motion-planner:build), GPU_IMG (motion-planner:gpu), CUROBO_SPHERE_ARGS (generate_curobo_robot.py args;
# default `--ignore-hops 2`, the same rule as the 6-joint `robot_assets.py` curobo stage; set it empty to get cuRobo's
# sampled matrix alone, add `--ignore-group a,b,c` / `--ignore-pair a:b` / `--sphere-buffer M`).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
PROFILE=$REPO/src/description/mobile_manipulator_description/config/profiles/ur10e_telescopic.yaml
OUT=$REPO/build/robot_assets/ur10e_telescopic
TOOL=tcp_link
SKIP_URDF=0
BUILD_IMG=${BUILD_IMG:-motion-planner:build}
GPU_IMG=${GPU_IMG:-motion-planner:gpu}

while [ $# -gt 0 ]; do
  case $1 in
    --profile) PROFILE=$2; shift 2;;
    --out) OUT=$2; shift 2;;
    --tool-frame) TOOL=$2; shift 2;;
    --skip-urdf) SKIP_URDF=1; shift;;
    -h|--help) sed -n "2,15p" "$0"; exit 0;;
    *) echo "unknown argument: $1" >&2; exit 2;;
  esac
done
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
[ -f "$PROFILE" ] || { echo "profile not found: $PROFILE" >&2; exit 2; }

if [ "$SKIP_URDF" = 0 ]; then
  "$HERE/generate_robot_assets.sh" "$PROFILE" --out "$OUT" --only urdf
fi
[ -f "$OUT/robot.urdf" ] || { echo "$OUT/robot.urdf missing (run without --skip-urdf)" >&2; exit 1; }

# ROS environment of the build image with the description package visible to the URI resolver, as robot_assets.py does
# (same SOURCE_ENV): package://mobile_manipulator_description resolves to the repo mounted at /repo.
SOURCE_ENV='source /opt/ros/*/setup.bash >/dev/null;
mkdir -p /tmp/pfx/share/ament_index/resource_index/packages;
ln -sfn /repo/src/description/mobile_manipulator_description /tmp/pfx/share/mobile_manipulator_description;
touch /tmp/pfx/share/ament_index/resource_index/packages/mobile_manipulator_description;
export AMENT_PREFIX_PATH=/tmp/pfx:$AMENT_PREFIX_PATH;'

docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 \
  -v "$REPO:/repo:ro" -v "$OUT:$OUT" -w "$OUT" "$BUILD_IMG" bash -c "$SOURCE_ENV"'
set -e
python3 /repo/docker/tools/resolve_package_uris.py "$1/robot.urdf" "$1/full_resolved"
python3 /repo/docker/tools/primitives_to_meshes.py "$1/full_resolved/robot.urdf" "$1/meshed"
# primitives_to_meshes copies only its own STLs: bring the package meshes too
cp -r "$1/full_resolved/ur_description" "$1/meshed/"' _ "$OUT"

# shellcheck disable=SC2086  # CUROBO_SPHERE_ARGS is a word list on purpose
docker run --rm --gpus all --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$REPO/docker/tools:/tools:ro" -v "$OUT:$OUT" -w "$OUT" \
  "$GPU_IMG" /opt/venv/bin/python /tools/generate_curobo_robot.py --urdf "$OUT/meshed/robot.urdf" \
  --tool-frame "$TOOL" --out "$OUT/lift_arm.yml" ${CUROBO_SPHERE_ARGS---ignore-hops 2}

echo "wrote $OUT/lift_arm.yml"
python3 - "$OUT/lift_arm.yml" <<'PY'
import sys, yaml


def find(node):
    if isinstance(node, dict):
        if isinstance(node.get('joint_names'), list):
            return node['joint_names']
        for v in node.values():
            r = find(v)
            if r:
                return r
    return None


names = find(yaml.safe_load(open(sys.argv[1])))
print('joint_names:', names)
if not names or len(names) != 7 or names[0] != 'lift_joint':
    sys.exit('expected 7 joints starting with lift_joint')
PY
