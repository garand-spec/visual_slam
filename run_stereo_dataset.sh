#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
ENV_ROOT="${VISLAM_ENV_ROOT:-${CONDA_PREFIX:-/home/linaro/miniconda3/envs/fire-monitor}}"
ORB_ROOT="${VISLAM_ORB_ROOT:-$PROJECT_ROOT/vendor/ORB_SLAM3}"
OPENCV_ROOT="${VISLAM_OPENCV_ROOT:-$PROJECT_ROOT/vendor/opencv_cuda/install}"
VOCABULARY="$ORB_ROOT/Vocabulary/ORBvoc.txt"
ARCHIVE="$ORB_ROOT/Vocabulary/ORBvoc.txt.tar.gz"
OLD_LD_LIBRARY_PATH="$(printenv LD_LIBRARY_PATH || true)"

if [[ ! -x "$PROJECT_ROOT/bin/orbslam3_stereo_dataset" ]]; then
    echo "Dataset runner is not built. Run ./build_stereo_dataset.sh first." >&2
    exit 2
fi
if [[ ! -f "$VOCABULARY" && -f "$ARCHIVE" ]]; then
    echo "Extracting ORB vocabulary..."
    tar -xzf "$ARCHIVE" -C "$(dirname "$ARCHIVE")"
fi
if [[ ! -f "$VOCABULARY" ]]; then
    echo "ORB vocabulary not found: $VOCABULARY" >&2
    exit 2
fi

export LD_LIBRARY_PATH="$OPENCV_ROOT/lib:$ORB_ROOT/lib:$ORB_ROOT/Thirdparty/g2o/lib:$ORB_ROOT/Thirdparty/DBoW2/lib:$ENV_ROOT/lib:$OLD_LD_LIBRARY_PATH"
exec "$PROJECT_ROOT/bin/orbslam3_stereo_dataset" \
  --vocabulary "$VOCABULARY" "$@"
