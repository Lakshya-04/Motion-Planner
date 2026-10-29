#!/usr/bin/env bash
# Build the upstream robot-model generation tools (FOAM: URDF -> spheres, Cricket: spheres ->
# VAMP header) as their own images from their own Dockerfiles, at pinned commits.
# They are kept out of the main image because they pull a heavy, unrelated toolchain
# (Pinocchio + CppAD + CGAL, or conda). Usage: docker/tools/build_robot_tools.sh [workdir]
set -euo pipefail

FOAM_REF=116928f71aaa     # CoMMALab/foam, MIT
CRICKET_REF=a2e664f97b6a  # CoMMALab/cricket, MIT
WORK="${1:-$HOME/.cache/motion-planner-tools}"
mkdir -p "$WORK"

build() {
  local name=$1 repo=$2 ref=$3
  [ -d "$WORK/$name" ] || git clone "https://github.com/CoMMALab/$repo.git" "$WORK/$name"
  git -C "$WORK/$name" checkout -q "$ref"
  docker build -t "motion-planner-$name:$ref" "$WORK/$name"
  echo "built motion-planner-$name:$ref"
}

# FOAM: our own thin Dockerfile (upstream's is a developer shell that expects a missing file).
docker build -f "$(dirname "$0")/foam.Dockerfile" --build-arg FOAM_REF="$FOAM_REF" \
  -t "motion-planner-foam:$FOAM_REF" "$(dirname "$0")"
echo "built motion-planner-foam:$FOAM_REF"
# Cricket: upstream's Dockerfile is a clean two-stage image whose entrypoint is fkcc_gen.
build cricket cricket "$CRICKET_REF"
