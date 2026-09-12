#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
COMPAT_LIB="$PROJECT_ROOT/vendor/rervision_single_imu/native/libgst_compat.so"
OLD_LD_LIBRARY_PATH="$(printenv LD_LIBRARY_PATH || true)"
export SCAM_GST_DECODER=jpegdec
if [[ -z "${DISPLAY:-}" ]]; then export DISPLAY=:1; fi
if [[ -z "${XAUTHORITY:-}" && -f /run/user/1000/gdm/Xauthority ]]; then export XAUTHORITY=/run/user/1000/gdm/Xauthority; fi
export LD_LIBRARY_PATH="$PROJECT_ROOT/vendor/rervision_single_imu/native/build:/usr/local/lib:/usr/lib/aarch64-linux-gnu:$OLD_LD_LIBRARY_PATH"
exec env LD_PRELOAD="$COMPAT_LIB" "$PROJECT_ROOT/bin/calibration_capture_cpp" "$@"
