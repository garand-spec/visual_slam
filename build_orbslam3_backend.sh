#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
ENV_ROOT="${VISLAM_ENV_ROOT:-${CONDA_PREFIX:-/home/linaro/miniconda3/envs/fire-monitor}}"
ORB_ROOT="${VISLAM_ORB_ROOT:-$PROJECT_ROOT/vendor/ORB_SLAM3}"
OPENCV_ROOT="${VISLAM_OPENCV_ROOT:-$PROJECT_ROOT/vendor/opencv_cuda/install}"
cmake -S "$ORB_ROOT" -B "$ORB_ROOT/build_modeling" \
  -DCMAKE_BUILD_TYPE=Release -DORB_SLAM3_USE_CUDA=ON \
  -DOpenCV_DIR="$OPENCV_ROOT/lib/cmake/opencv4" \
  -DCMAKE_PREFIX_PATH="$ENV_ROOT" -DCMAKE_BUILD_RPATH="$OPENCV_ROOT/lib" \
  -DCMAKE_SHARED_LINKER_FLAGS="-L$ENV_ROOT/lib"
cmake --build "$ORB_ROOT/build_modeling" --target ORB_SLAM3 -j4
