# RERVISION 官方图像 + IMU 接入

本目录保留厂商提供的 Linux 单路图像+IMU Python SDK，并使用 NVIDIA Jetson ARM64 版本 libscam.so。

## 已配置内容

- 官方 ARM64 SDK：vendor/rervision_single_imu/python/build/libscam.so
- 官方 ctypes 封装：vendor/rervision_single_imu/python/scam_sdk.py
- 官方图像+IMU 示例：vendor/rervision_single_imu/python/print_imu.py
- Jetson GStreamer 兼容库：vendor/rervision_single_imu/native/libgst_compat.so

摄像头是 USB UVC 双目鱼眼设备，当前官方 SDK 将一帧同步图像输出为横向拼接图：

- 设备格式：4000x1200 @ 60 fps（摄像头枚举值）
- SDK 解码输出：3840x1200 RGB24
- 左右目：后续按宽度对半切分，各 1920x1200
- 每帧 IMU：11 组 ICM42688 加速度/陀螺仪数据
- 每帧还带 5 组 AK09940 磁力计/温度状态数据
- 图像和 IMU 使用厂商帧内时间戳关联

## 启动官方示例

    cd /home/linaro/projects/fire_monitor
    ./run_scam_imu.sh

指定 NV12 输出（适合直接读取 Y 平面）：

    ./run_scam_imu.sh --device 0 --format-index 0 --sdk-format NV12

指定 RGB24 输出（当前实测约 41.5 FPS，适合先做双目预览和 SLAM 接口）：

    ./run_scam_imu.sh --device 0 --format-index 0 --sdk-format RGB24

## 为什么需要兼容库

厂商 libscam.so 内部写死了旧版 GStreamer 管线：

    appsrc ! jpegparse ! nvv4l2decoder mjpeg=true ! nvvidconv ! ...

当前 Jetson 系统中的 nvv4l2decoder 已没有 mjpeg 属性，并且这路 4000x1200 MJPEG 不被当前硬件解码器接受。因此 libgst_compat.so 仅拦截官方库创建该管线的调用，把解码器替换为：

    appsrc ! jpegparse ! jpegdec ! videoconvert ! ...

设备枚举、V4L2 采集、官方回调、图像格式、帧时间戳和 IMU 数据结构仍全部由厂商 SDK 负责。未设置 LD_PRELOAD 时，官方库仍会复现 GStreamer MJPEG decoder init failed。

## 后续 SLAM 接口

SLAM 应从 CamData/FrameData 读取：

1. 按 3840 宽度切成左右 1920x1200 图像；
2. 使用左右目各自的鱼眼内参和畸变参数进行校正；
3. 使用 startExpouseTime 或 endExpouseTime 与 11 组 IMU 时间戳关联；
4. 先确认 IMU 坐标轴、单位和相机-IMU 外参，再接入 VIO/SLAM。
