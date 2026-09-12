#!/usr/bin/env bash
set -eo pipefail
PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
PYTHON="/home/linaro/miniconda3/envs/fire-monitor/bin/python"
exec "$PYTHON" "$PROJECT_ROOT/src/calibrate_fisheye.py" "$@"
