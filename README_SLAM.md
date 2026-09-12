# RERVISION 双目鱼眼视觉 SLAM

当前主线是双目鱼眼稠密视觉 SLAM：ORB-SLAM3 负责实时定位和稀疏几何骨架，异步鱼眼校正与 SGBM 深度线程负责生成稠密点云并按相机位姿融合。默认使用 Jetson GPU 加速 ORB 特征提取；IMU 模式保留为可选功能。

摄像头原始拼接画面的左右位置与旧程序标签相反。默认启动现在把原始第二半幅作为 `LEFT/Camera1`，并自动加载成对交换后的内参和逆外参，不能只交换画面而继续使用旧标定。

## 默认启动

在 Jetson 桌面终端或 SSH 会话中执行：

    cd /home/linaro/projects/visual_slam
    ./run_visual_slam_3d.sh

程序默认持续运行到按 Ctrl+C，并显示 `ORB-SLAM3 RERVISION`：左右目当前画面、成功跟踪点、状态和恢复建议。按 q 或 ESC 也可正常退出。

查看 ORB-SLAM3 实时定位骨架时执行：

    ./run_visual_slam_3d.sh --pangolin-viewer

此模式显示 `ORB-SLAM3: 3D Sparse Map`，左侧面板给出跟踪状态；鼠标左键旋转，滚轮缩放。当前 Jetson X11/Pangolin 路径会与地图线程争用，特别是在反复重建地图时实测追踪约 2 FPS，因此该模式用于低速检查地图结构。默认 OpenCV 窗口实测约 21 FPS，适合正常采集和建图。

Pangolin 窗口仍显示定位用的稀疏骨架；完整稠密结果在正常退出时保存为 `dense_map.ply`。把稀疏定位前端直接替换成逐像素算法会破坏实时性，因此项目采用“稀疏定位 + 异步稠密融合”的标准结构。

## 后台启停

    ./start_slam.sh
    ./status_slam.sh
    ./stop_slam.sh

`stop_slam.sh` 会先发送 Ctrl+C 等价信号，让程序正常导出轨迹和地图。后台日志位于 `data/runtime/orbslam3/slam.log`。

## 常用选项

    ./run_visual_slam_3d.sh --imu
    ./run_visual_slam_3d.sh --no-viewer --seconds 30
    ./run_visual_slam_3d.sh --pangolin-viewer
    ./run_visual_slam_3d.sh --dense-interval 4 --dense-voxel 0.03

- `--imu`：启用双目视觉 + IMU。
- `--seconds N`：运行 N 秒；0 表示持续运行。
- `--no-viewer`：关闭所有窗口，用于性能测试或后台采集。
- `--opencv-viewer`：显示低延迟双目状态窗口（默认）。
- `--pangolin-viewer`：切换到三维稀疏地图窗口。
- `--dense` / `--no-dense`：启用或关闭异步稠密建图；默认启用。
- `--dense-interval N`：每 N 个跟踪帧融合一次深度，默认 6；减小会更密但占用更多 CPU。
- `--dense-width N`：稠密立体校正宽度，默认 640，高度按 16:10 自动计算。
- `--dense-stride N`：每隔 N 个像素采样进体素地图，默认 2。
- `--dense-voxel M`：融合体素边长（米），默认 0.05；室内精细模型可尝试 0.03。
- `--dense-min-depth M`、`--dense-max-depth M`：有效深度范围，默认 0.3～12 米。
- `--swap-eyes`：使用修正后的左右顺序和交换标定（默认）。
- `--no-swap-eyes`：仅用于诊断旧顺序，会自动加载旧标定文件。

## 看懂状态

- `INITIALIZING`：尚未建立初始地图。让两个镜头都看到有纹理、远近层次明显的区域，并缓慢平移。
- `TRACKING OK`：定位正常。绿色点是当前成功跟踪的视觉特征。
- `TRACKING WEAK`：匹配不足、运动过快或场景退化。立即减速并朝已走过的区域移动。
- `TRACKING LOST`：定位已丢失。回到之前成功建图的视角，避免白墙、反光、运动模糊和遮挡任一镜头。

地图点数量不是越多越好；更重要的是 `TRACKING OK` 能持续、轨迹不跳变、地图结构在不同视角下保持稳定。

## 运行输出

每次运行创建：

    data/runtime/orbslam3/run_YYYYMMDD_HHMMSS/

- `dense_map.ply`：经过位姿和体素融合的稠密灰度点云，优先用 MeshLab 或 CloudCompare 查看。
- `map.ply`：ORB-SLAM3 过滤后的稀疏定位骨架，用于诊断和对照。
- `poses.csv`：每帧状态、地图点数量和相机位姿。
- `tracking_health.csv`：状态名、特征数、处理时延、连续丢失帧数等诊断数据。
- `status.txt`：当前跟踪健康状态，供 `status_slam.sh` 读取。
- `CameraTrajectory.txt`、`KeyFrameTrajectory.txt`：ORB-SLAM3 轨迹。

程序必须正常退出才能完整写出 `dense_map.ply`、`map.ply` 和轨迹文件。`status.txt` 中的 `dense_points` 是当前融合体素数量；只有跟踪状态为 `OK` 时才提交稠密帧，避免把丢失定位后的错误深度写进全局地图。

## 获得稳定地图

- 启动后先缓慢横向平移，不要只在原地快速旋转。
- 两个鱼眼镜头必须同时无遮挡，并保持标定时的相对位置。
- 优先拍摄桌角、门框、设备边缘和纹理丰富的墙面；纯白墙、重复纹理和强反光容易丢失。
- 运动过程中尽量让已有区域继续保留在视野中，形成闭环后再扩展到新区域。
- 相机输入为 3840x1200，左右目各 1920x1200；标定板为 9x12 方格，对应 8x11 内角点。

## 性能路径

相机使用 `jpegdec` 解码，再由 Jetson `nvvidconv` 转换为 NV12；SLAM 直接取 NV12 的 Y 平面，灰度显示是正常现象。ORB 特征提取使用项目内 OpenCV 4.12 CUDA，左右目各有独立 CUDA ORB 实例。稠密 SGBM、位姿估计和图优化仍走 CPU；当前 OpenCV 构建没有 `cudastereo`，所以稠密线程采用最新帧覆盖队列，慢一帧时直接丢弃旧任务，不会产生逐渐增加的相机延迟。

左右目 CUDA ORB 使用各自的非默认 CUDA Stream，并复用 `GpuMat`、主机描述子矩阵和关键点容器，避免左右目在 `Stream::Null()` 上串行以及逐帧重复分配 GPU 内存。Jetson 当前已启用 120W 模式与 `jetson_clocks`；重启后如需重新锁定 CPU/GPU/EMC 最高频率，执行：

    sudo jetson_clocks

同一套 960x600、1200 特征配置中，稀疏前端平均跟踪约 16～19 ms；默认 640x400、每 6 帧一次的异步稠密融合实机约 30 FPS。4 秒静态测试导出约 6 万个融合体素、约 2.1 MB 的 `dense_map.ply`。点数会随场景纹理、距离和有效运动变化。由于 CUDA 核只负责 ORB 前端，GPU 利用率不会持续满载；性能目标是降低端到端帧时延，而不是刻意让 GPU 显示 100%。
