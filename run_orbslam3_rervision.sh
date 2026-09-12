#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
ENV_ROOT="${VISLAM_ENV_ROOT:-${CONDA_PREFIX:-/home/linaro/miniconda3/envs/fire-monitor}}"
ORB_ROOT="${VISLAM_ORB_ROOT:-$PROJECT_ROOT/vendor/ORB_SLAM3}"
OPENCV_ROOT="${VISLAM_OPENCV_ROOT:-$PROJECT_ROOT/vendor/opencv_cuda/install}"
COMPAT_LIB="$PROJECT_ROOT/vendor/rervision_single_imu/native/libgst_compat.so"
OLD_LD_LIBRARY_PATH="$(printenv LD_LIBRARY_PATH || true)"

if [[ ! -x "$PROJECT_ROOT/bin/orbslam3_rervision" ]]; then
  echo "Realtime runner is not built. Run ./build_orbslam3_rervision.sh first." >&2
  exit 2
fi
if [[ ! -f "$ORB_ROOT/Vocabulary/ORBvoc.txt" && -f "$ORB_ROOT/Vocabulary/ORBvoc.txt.tar.gz" ]]; then
  echo "Extracting ORB vocabulary..."
  tar -xzf "$ORB_ROOT/Vocabulary/ORBvoc.txt.tar.gz" -C "$ORB_ROOT/Vocabulary"
fi
if [[ ! -f "$ORB_ROOT/Vocabulary/ORBvoc.txt" ]]; then
  echo "ORB vocabulary not found under $ORB_ROOT/Vocabulary" >&2
  exit 2
fi
if [[ ! -f "$COMPAT_LIB" ]]; then
  echo "Camera compatibility library not found: $COMPAT_LIB" >&2
  exit 2
fi

export SCAM_GST_DECODER=jpegdec
export VISLAM_MODEL_PYTHON="${VISLAM_MODEL_PYTHON:-$ENV_ROOT/bin/python}"
# The adapter owns the lightweight stereo/status window. Keeping OpenCV GUI
# calls out of Pangolin's render thread avoids severe X11/GTK lock contention.
export ORB_SLAM3_FRAME_VIEWER=0
if [[ -z "${DISPLAY:-}" ]]; then
  export DISPLAY=:1
fi
if [[ -z "${XAUTHORITY:-}" && -f /run/user/1000/gdm/Xauthority ]]; then
  export XAUTHORITY=/run/user/1000/gdm/Xauthority
fi
export LD_LIBRARY_PATH="$OPENCV_ROOT/lib:$ORB_ROOT/lib:$ORB_ROOT/Thirdparty/g2o/lib:$ORB_ROOT/Thirdparty/DBoW2/lib:$ENV_ROOT/lib:$PROJECT_ROOT/vendor/rervision_single_imu/native/build:/usr/local/lib:/usr/lib/aarch64-linux-gnu:$OLD_LD_LIBRARY_PATH"

exec env LD_PRELOAD="$COMPAT_LIB" "$PROJECT_ROOT/bin/orbslam3_rervision" "$@"
