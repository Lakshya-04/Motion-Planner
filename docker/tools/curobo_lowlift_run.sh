#!/usr/bin/env bash
# Reproduces the low-lift docked-to-stow check (docs/curobo_v2_notes.md): generates the 7-joint cuRobo config twice
# (`--ignore-hops 2`, the default, and cuRobo's sampled matrix alone) and runs curobo_lowlift_probe.py on each in
# motion-planner:gpu. One GPU job at a time: on this host queue it, do not run it by hand:
#   SIMQ_BY=<name> ~/.claude/coordination/motion-planner/simq_add.sh 30 curobo-lowlift "$PWD/docker/tools/curobo_lowlift_run.sh"
# VARIANTS (default 'hops2 sampled'; also hops2_pair = hops2 plus --ignore-pair base_link:lift_stage_2_link).
# Output: build/lowlift/{hops2,sampled}/probe.json and probe.log. Env: GPU_IMG, PROBE_ARGS (extra probe arguments),
# WAIT_MIN (minutes to wait for slots.sh to say node YES, default 60).
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
GPU_IMG=${GPU_IMG:-motion-planner:gpu}
OUT=$REPO/build/lowlift
mkdir -p "$OUT" "$HOME/.cache/motion-planner-curobo"

deadline=$(( $(date +%s) + ${WAIT_MIN:-60} * 60 ))
while [ "$(date +%s)" -lt "$deadline" ]; do
  bash "$HERE/hostwatch/slots.sh" | grep -q '^node *YES' && break
  sleep 15
done
bash "$HERE/hostwatch/slots.sh" | grep -q '^node *YES' || { echo "no node slot after ${WAIT_MIN:-60} min"; bash "$HERE/hostwatch/slots.sh"; exit 3; }

for variant in ${VARIANTS:-hops2 sampled}; do
  d=$OUT/$variant; mkdir -p "$d"
  if [ ! -f "$d/lift_arm.yml" ]; then
    case $variant in
      hops2) args='--ignore-hops 2';;
      sampled) args='';;
      hops2_pair) args='--ignore-hops 2 --ignore-pair base_link:lift_stage_2_link';;
      *) echo "unknown variant $variant"; continue;;
    esac
    echo "== generate $variant (CUROBO_SPHERE_ARGS='$args')"
    CUROBO_SPHERE_ARGS="$args" "$HERE/generate_curobo_7dof.sh" --out "$d" || { echo "generate $variant failed"; continue; }
  fi
  echo "== probe $variant"
  # shellcheck disable=SC2086
  docker run --rm --gpus all --user "$(id -u):$(id -g)" -e HOME=/cache -e PYTHONDONTWRITEBYTECODE=1 \
    -v "$HOME/.cache/motion-planner-curobo:/cache" -v "$REPO:/repo:ro" -v "$d:$d" \
    "$GPU_IMG" /opt/venv/bin/python /repo/docker/tools/curobo_lowlift_probe.py --robot "$d/lift_arm.yml" \
    --out "$d/probe.json" ${PROBE_ARGS:-} 2>&1 | tee "$d/probe.log"
done
echo done
