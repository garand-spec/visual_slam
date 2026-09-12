#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
ENV_ROOT="${VISLAM_ENV_ROOT:-${CONDA_PREFIX:-/home/linaro/miniconda3/envs/fire-monitor}}"
ORB_ROOT="${VISLAM_ORB_ROOT:-$PROJECT_ROOT/vendor/ORB_SLAM3}"
OPENCV_ROOT="${VISLAM_OPENCV_ROOT:-$PROJECT_ROOT/vendor/opencv_cuda/install}"

for required in "$ORB_ROOT/include/System.h" "$ORB_ROOT/lib/libORB_SLAM3.so" \
                "$OPENCV_ROOT/include/opencv4" "$ENV_ROOT/include/eigen3"; do
    if [[ ! -e "$required" ]]; then
        echo "Missing build dependency: $required" >&2
        exit 2
    fi
done

mkdir -p "$PROJECT_ROOT/bin"
g++ -std=c++17 -O3 -pthread \
  -I"$OPENCV_ROOT/include/opencv4" -I"$ENV_ROOT/include" -I"$ENV_ROOT/include/eigen3" \
  -I"$ORB_ROOT" -I"$ORB_ROOT/include" -I"$ORB_ROOT/include/CameraModels" \
  -I"$ORB_ROOT/Thirdparty/Sophus" \
  "$PROJECT_ROOT/src/orbslam3_stereo_dataset.cpp" \
  -L"$ORB_ROOT/lib" -Wl,-rpath,"$ORB_ROOT/lib" \
  -L"$ORB_ROOT/Thirdparty/g2o/lib" -Wl,-rpath,"$ORB_ROOT/Thirdparty/g2o/lib" \
  -L"$ORB_ROOT/Thirdparty/DBoW2/lib" -Wl,-rpath,"$ORB_ROOT/Thirdparty/DBoW2/lib" \
  -L"$OPENCV_ROOT/lib" -Wl,-rpath,"$OPENCV_ROOT/lib" \
  -L"$ENV_ROOT/lib" -Wl,-rpath,"$ENV_ROOT/lib" \
  -lORB_SLAM3 -lboost_serialization -lcrypto \
  -lopencv_imgcodecs -lopencv_calib3d -lopencv_features2d \
  -lopencv_imgproc -lopencv_core \
  -o "$PROJECT_ROOT/bin/orbslam3_stereo_dataset"

echo "Built $PROJECT_ROOT/bin/orbslam3_stereo_dataset"
