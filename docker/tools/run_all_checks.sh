#!/usr/bin/env bash
# Host-only checks: no ROS, no containers. Run from anywhere.
set -euo pipefail
cd "$(dirname "$0")/../.."
python3 docker/tools/check_public_readiness.py
python3 docker/tools/check_params.py
python3 docker/tools/check_doc_citations.py
python3 -m pytest -q docker/tools/tests/test_check_public_readiness.py \
  docker/tools/tests/test_check_params.py docker/tools/tests/test_check_doc_citations.py
