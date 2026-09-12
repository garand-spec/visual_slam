#!/usr/bin/env bash
set -eo pipefail
PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
PYTHON="/home/linaro/miniconda3/envs/fire-monitor/bin/python"
COMPAT_LIB="$PROJECT_ROOT/vendor/rervision_single_imu/native/libgst_compat.so"
export SCAM_GST_DECODER=jpegdec
if [[ -z "$DISPLAY" ]]; then export DISPLAY=:1; fi
if [[ -z "$XAUTHORITY" && -f /run/user/1000/gdm/Xauthority ]]; then export XAUTHORITY=/run/user/1000/gdm/Xauthority; fi
exec env LD_PRELOAD="$COMPAT_LIB" "$PYTHON" "$PROJECT_ROOT/src/capture_calibration.py" "$@"
