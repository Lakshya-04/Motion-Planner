#!/usr/bin/env bash
# Publish the prebuilt images to GitHub Container Registry by hand (the project has no CI).
#
#   docker/tools/publish_images.sh                 dry run: print what would be tagged and pushed
#   docker/tools/publish_images.sh --push          log in check, tag, push deps build sim
#   docker/tools/publish_images.sh --push --gpu    also push the GPU image (about 12 GB)
#   docker/tools/publish_images.sh --push --version v0.1.0   tag <version> as well as `latest`
#
# Needs: the images built locally (docker/Dockerfile targets; see docs/docker.md), and a login with the
# write:packages scope: `gh auth refresh -s write:packages` then
# `gh auth token | docker login ghcr.io -u <github user> --password-stdin`.
# Images are tagged ghcr.io/<owner>/motion-planner:<target>[-<version>]. Make the package public once in
# the GitHub UI (Packages > motion-planner > settings) so a pull needs no login.
set -euo pipefail
OWNER=${GHCR_OWNER:-lakshya-04}
REPO=ghcr.io/${OWNER}/motion-planner
TARGETS=(deps build sim)
PUSH=0; VERSION=""
while [ $# -gt 0 ]; do
  case $1 in
    --push) PUSH=1; shift;;
    --gpu) TARGETS+=(full); shift;;
    --version) VERSION=$2; shift 2;;
    -h|--help) sed -n 2,14p "$0"; exit 0;;
    *) echo "unknown argument $1" >&2; exit 2;;
  esac
done
for t in "${TARGETS[@]}"; do
  src=motion-planner:$t
  docker image inspect "$src" > /dev/null 2>&1 || { echo "missing local image $src (build it first)" >&2; exit 1; }
  size=$(docker image inspect "$src" --format '{{.Size}}' | awk '{printf "%.1f GB", $1/1e9}')
  tags=("$REPO:$t")
  [ -n "$VERSION" ] && tags+=("$REPO:$t-$VERSION")
  for tag in "${tags[@]}"; do
    echo "$src ($size) -> $tag"
    if [ $PUSH = 1 ]; then docker tag "$src" "$tag" && docker push "$tag"; fi
  done
done
[ $PUSH = 1 ] || echo "dry run: nothing tagged or pushed (use --push)"
