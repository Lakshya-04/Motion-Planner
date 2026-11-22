#!/usr/bin/env bash
# Print ccache statistics (hit rate) for a cache dir, using the ccache inside an image.
#   docker/tools/ccache_stats.sh [CACHE_DIR] [IMAGE]
# CACHE_DIR defaults to $MP_CCACHE_DIR or $HOME/.cache/mp-ccache; IMAGE to motion-planner:dev.
# If a host ccache exists it is used when docker is not available.
set -euo pipefail
DIR="${1:-${MP_CCACHE_DIR:-$HOME/.cache/mp-ccache}}"
IMAGE="${2:-motion-planner:dev}"
[ -d "$DIR" ] || { echo "no such cache dir: $DIR" >&2; exit 1; }
if command -v docker >/dev/null 2>&1; then
  docker run --rm -v "$DIR:/ccache" -e CCACHE_DIR=/ccache --entrypoint ccache "$IMAGE" -s
elif command -v ccache >/dev/null 2>&1; then
  CCACHE_DIR="$DIR" ccache -s
else
  echo "need docker or ccache" >&2; exit 1
fi
