#!/usr/bin/env bash
# Every per-robot artefact from one profile: URDF, SRDF + allowed-collision matrix, FOAM spheres,
# VAMP header, cuRobo config, reachability chain + map, self-collision sanity, manifest.
#
#   docker/tools/generate_robot_assets.sh <profile.yaml> [--out DIR] [--only a,b] [--skip a,b]
#   docker/tools/generate_robot_assets.sh --list          # stage names
#
# Stages: urdf srdf spheres vamp curobo reach check manifest. Each runs in the image it needs
# (motion-planner:build, -foam, -cricket, :gpu); see docs/robot_assets.md. Default output:
# build/robot_assets/<ProfileName>. Env: CRICKET_SRC (pinned Cricket checkout with resources/templates),
# BUILD_IMG GPU_IMG FOAM_IMG CRICKET_IMG, FOAM_ARGS.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
exec python3 "$HERE/robot_assets.py" "$@"
