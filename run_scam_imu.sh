#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
PYTHON="/home/linaro/miniconda3/envs/fire-monitor/bin/python"
SDK_ROOT="$PROJECT_ROOT/vendor/rervision_single_imu/python"
COMPAT_LIB="$PROJECT_ROOT/vendor/rervision_single_imu/native/libgst_compat.so"
export SCAM_GST_DECODER=jpegdec

if [[ ! -x "$PYTHON" ]]; then
  echo "fire-monitor Conda environment not found: $PYTHON" >&2
  exit 1
fi
if [[ ! -f "$SDK_ROOT/print_imu.py" ]]; then
  echo "official RERVISION Python sample not found: $SDK_ROOT/print_imu.py" >&2
  exit 1
fi
if [[ ! -f "$COMPAT_LIB" ]]; then
  echo "GStreamer compatibility library not found: $COMPAT_LIB" >&2
  exit 1
fi

cd "$SDK_ROOT"
if (( $# == 0 )); then
  set -- --device 0 --format-index 0 --sdk-format RGB24
fi

exec env LD_PRELOAD="$COMPAT_LIB" "$PYTHON" "$SDK_ROOT/print_imu.py" "$@"
