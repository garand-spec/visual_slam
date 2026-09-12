#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="$HOME/miniconda3/envs/fire-monitor/bin/python"
if [[ ! -x "$PYTHON" ]]; then
  echo "fire-monitor Conda environment not found: $PYTHON" >&2
  exit 1
fi
exec "$PYTHON" "$PROJECT_ROOT/src/camera_preview.py" "$@"
