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
        echo "Override roots with VISLAM_ENV_ROOT, VISLAM_ORB_ROOT and VISLAM_OPENCV_ROOT." >&2
        exit 2
    fi
done

mkdir -p "$PROJECT_ROOT/bin"
g++ -std=c++17 -O3 -pthread   -DHAVE_EIGEN -DHAVE_EPOXY -D_LINUX_ '-DPANGO_DEFAULT_WIN_URI="x11"'   -I"$OPENCV_ROOT/include/opencv4" -I"$ENV_ROOT/include" -I"$ENV_ROOT/include/eigen3"   -I"$ORB_ROOT" -I"$ORB_ROOT/include" -I"$ORB_ROOT/include/CameraModels"   -I"$ORB_ROOT/Thirdparty/Sophus"   -I"$PROJECT_ROOT/vendor/rervision_single_imu/native"   "$PROJECT_ROOT/src/orbslam3_rervision.cpp"   -L"$ORB_ROOT/lib" -Wl,-rpath,"$ORB_ROOT/lib"   -L"$ORB_ROOT/Thirdparty/g2o/lib" -Wl,-rpath,"$ORB_ROOT/Thirdparty/g2o/lib"   -L"$ORB_ROOT/Thirdparty/DBoW2/lib" -Wl,-rpath,"$ORB_ROOT/Thirdparty/DBoW2/lib"   -L"$OPENCV_ROOT/lib" -Wl,-rpath,"$OPENCV_ROOT/lib"   -L"$ENV_ROOT/lib" -Wl,-rpath,"$ENV_ROOT/lib"   -L"$PROJECT_ROOT/vendor/rervision_single_imu/native/build"   -Wl,-rpath,"$PROJECT_ROOT/vendor/rervision_single_imu/native/build"   -Wl,--no-as-needed   -lORB_SLAM3 -lscam -lboost_serialization -lcrypto   -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_calib3d -lopencv_features2d -lopencv_imgproc -lopencv_core -lopencv_cudafeatures2d -lopencv_cudafilters -lopencv_cudaimgproc -lopencv_cudaarithm -lopencv_cudawarping   -lpango_glgeometry -lpango_plot -lpango_python -lpango_scene -lpango_tools   -lpango_video -lpango_geometry -ltinyobj -lpango_display -lpango_vars   -lpango_windowing -lpango_opengl -lEGL -lOpenGL -lepoxy -lpango_image   -lpango_packetstream -lpango_core -lrt   -o "$PROJECT_ROOT/bin/orbslam3_rervision"

echo "Built $PROJECT_ROOT/bin/orbslam3_rervision"
