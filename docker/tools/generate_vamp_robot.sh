#!/usr/bin/env bash
# URDF + SRDF -> spherized URDF (FOAM) -> VAMP robot header (Cricket).
#
#   docker/tools/generate_vamp_robot.sh <name> <robot.urdf> <robot.srdf|-> <end_effector_link> <out_dir> [resolution]
#
# Pass "-" for the SRDF to let Cricket derive the allowed collision pairs itself by sampling the
# spherized model (pairs that always or never collide). That is the consistent choice when the SRDF
# was computed for the original primitive/mesh model: spherization bulges, so pairs that were free in
# the source model can overlap in the sphere model and make every configuration invalid.
#
# <name> becomes the C++ struct name in the header, so it must be a valid identifier (no '-').
# The URDF's collision geometry may be primitives or package-relative-free meshes; primitives are
# converted to meshes first because FOAM only spherizes meshes. Needs the two tool images from
# docker/tools/build_robot_tools.sh. FOAM_ARGS passes extra options to FOAM (e.g. FOAM_ARGS="--branch 16"
# for more spheres per link; the default below, see FOAM_ARGS). FOAM_ONLY=1 stops after the spherized URDF; SKIP_FOAM=1 reuses an existing
# spherized.urdf in <out_dir> and only runs Cricket (docker/tools/robot_assets.py splits the two so each
# can be skipped). The header is large: write it to a build directory, do not
# commit it.
set -euo pipefail
[ $# -ge 5 ] || { sed -n 2,11p "$0"; exit 2; }
NAME=$1; URDF=$(realpath "$2"); SRDF=$3; [ "$SRDF" = "-" ] || SRDF=$(realpath "$SRDF"); EE=$4; OUT=$(mkdir -p "$5" && realpath "$5"); RES=${6:-32}
HERE=$(cd "$(dirname "$0")" && pwd)
FOAM_IMG=${FOAM_IMG:-motion-planner-foam:116928f71aaa}
CRICKET_IMG=${CRICKET_IMG:-motion-planner-cricket:a2e664f97b6a}
# FOAM caps a link's sphere count by how well one bounding sphere fits it, so the 0.04 m thick lift
# carriage plate (the mount the arm sits on) got 6 spheres of radius 0.08: they bulge 6 cm above the
# plate and touched the upper arm at stand_left / stand_right / dock_1..3 (60 of 72 benchmark endpoints
# rejected) although the mesh clears it by more than 10 cm. A per-link multiplier (4x) gives 23 spheres
# of radius <= 0.044 and the pair is checked honestly. Links the URDF does not have are ignored.
FOAM_ARGS=${FOAM_ARGS---lift_carriage_link=4}
# The Cricket image carries only the fkcc_gen binary; the code templates come from the pinned
# checkout that build_robot_tools.sh cloned.
CRICKET_SRC=${CRICKET_SRC:-$HOME/.cache/motion-planner-tools/cricket}
[ -d "$CRICKET_SRC/resources/templates" ] || { echo "Cricket templates not found in $CRICKET_SRC (set CRICKET_SRC)" >&2; exit 2; }
[[ $NAME =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]] || { echo "name '$NAME' is not a valid C++ identifier" >&2; exit 2; }

if [ -z "${SKIP_FOAM:-}" ]; then
mkdir -p "$OUT/meshed"
python3 "$HERE/primitives_to_meshes.py" "$URDF" "$OUT/meshed"

# Mesh paths in the URDF are resolved relative to the URDF's directory by FOAM; copy any meshes the
# original URDF references with relative paths next to the converted one.
python3 - "$URDF" "$OUT/meshed" <<'PY'
import pathlib, shutil, sys, xml.etree.ElementTree as ET
src, dst = pathlib.Path(sys.argv[1]).parent, pathlib.Path(sys.argv[2])
for m in ET.parse(sys.argv[1]).getroot().iter('mesh'):
    f = m.get('filename', '')
    if f.startswith(('package://', 'file://', '/')):
        sys.exit(f"mesh '{f}' is not a relative path: resolve package:// URIs first")
    if (src / f).exists():
        (dst / f).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src / f, dst / f)
PY

docker run --rm --user "$(id -u):$(id -g)" -v "$OUT:/work" -w /work "$FOAM_IMG" \
  --filename meshed/robot.urdf --output spherized.urdf --database sphere_database.json ${FOAM_ARGS:-}
else
  [ -f "$OUT/spherized.urdf" ] || { echo "SKIP_FOAM set but $OUT/spherized.urdf is missing" >&2; exit 2; }
fi
if [ -n "${FOAM_ONLY:-}" ]; then echo "wrote $OUT/spherized.urdf"; exit 0; fi

rm -rf "$OUT/templates" && cp -r "$CRICKET_SRC/resources/templates" "$OUT/templates"
SRDF_LINE=""
if [ "$SRDF" != "-" ]; then cp "$SRDF" "$OUT/robot.srdf"; SRDF_LINE='"srdf": "robot.srdf",'; else rm -f "$OUT/robot.srdf"; fi
cat > "$OUT/cricket.json" <<JSON
{
  "name": "$NAME",
  "urdf": "spherized.urdf",
  $SRDF_LINE
  "end_effector": "$EE",
  "resolution": $RES,
  "template": "templates/fk_template.hh",
  "subtemplates": [{"name": "ccfk", "template": "templates/ccfk_template.hh"}],
  "output": "${NAME}.hh"
}
JSON
docker run --rm --user "$(id -u):$(id -g)" -v "$OUT:/mount" "$CRICKET_IMG" /mount/cricket.json -t /mount/${NAME}.hh
echo "wrote $OUT/${NAME}.hh"
