#!/usr/bin/env bash
# Check this machine for what the stack needs and say what to fix. Thin wrapper over doctor_lib.py.
#
#   docker/tools/doctor.sh [--quiet] [--json] [--gpu] [--repo] [--no-network]
#
#   --quiet       only warnings, failures and the summary
#   --json        machine-readable output
#   --gpu         also run `docker run --gpus all motion-planner:gpu nvidia-smi` (needs the gpu image)
#   --repo        also run the public-readiness scan
#   --no-network  skip the mirror and TLS checks
#
# Exit status is 1 when something failed, 0 otherwise (warnings do not count).
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if ! command -v python3 >/dev/null 2>&1; then
  echo "doctor needs python3 for its checks and found none on PATH; install it (apt install python3)." >&2
  exit 2
fi
exec python3 "$HERE/doctor_lib.py" "$@"
