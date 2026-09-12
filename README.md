# Visual SLAM：双目鱼眼定位与稠密建图

这是一套可直接部署的视觉 SLAM 工程，主线面向 RERVISION 双目鱼眼 + IMU 相机和 NVIDIA Jetson。它使用 ORB-SLAM3 完成双目定位、回环检测和全局图优化，并异步计算双目深度、融合稠密体素地图。项目同时提供不依赖相机的通用双目数据集回放入口，便于复现、调参和回归测试。

## 能力范围

- 双目视觉 SLAM：真实尺度定位、关键帧地图、回环和全局优化。
- 双目视觉惯性模式：接入相机随帧 IMU 数据；外参完成标定前应优先使用纯双目模式。
- 鱼眼相机：Kannala-Brandt 模型，内置本机 1920×1200/目真实标定。
- 稠密建图：异步鱼眼校正、SGBM 深度、位姿融合、体素去重，导出 PLY。
- 实时诊断：跟踪状态、特征数、地图点、连续丢失帧、帧时延和恢复建议。
- 离线复现：侧-by-侧视频转同步图片序列，用 ORB-SLAM3 数据集运行器回放。
- 工程运维：前后台启停、优雅退出、轨迹和地图导出、项目及结果自动校验。

## 一分钟启动（Jetson 实机）

```bash
cd /home/linaro/projects/visual_slam

# 首次部署或源码更新后构建
./build_orbslam3_rervision.sh

# 前台运行；q、Esc 或 Ctrl+C 会正常保存地图
./run_visual_slam_3d.sh
```

无显示器运行 30 秒：

```bash
./run_visual_slam_3d.sh --no-viewer --seconds 30
```

后台运行和查看状态：

```bash
./start_slam.sh --no-viewer
./status_slam.sh
./stop_slam.sh
```

`stop_slam.sh` 会先发送优雅退出信号，确保轨迹和点云写完。不要直接断电或强杀进程。

## 离线数据集工作流

这一流程适合在没有相机时做算法复现，也适合回放实机录制的视频。

1. 把左右并排视频拆为同步序列（RERVISION 原始画面应使用 `--swap-eyes`）：

```bash
python3 tools/prepare_stereo_dataset.py data/recording.mp4 \
  --output data/datasets/room01 --swap-eyes
```

输出目录包含 `left/`、`right/`、`timestamps.txt` 和 `dataset.json`。

2. 构建并运行通用数据集适配器：

```bash
./build_stereo_dataset.sh
./run_stereo_dataset.sh \
  --settings configs/orbslam3_rervision_stereo_inertial_swapped.yaml \
  --left data/datasets/room01/left \
  --right data/datasets/room01/right \
  --timestamps data/datasets/room01/timestamps.txt \
  --output data/runtime/offline/room01
```

输入图片尺寸必须与 YAML 中的 `Camera.width` / `Camera.height` 一致。其他相机必须使用自己的标定 YAML，不能套用仓库内的 RERVISION 参数。

## 常用实时参数

```bash
./run_visual_slam_3d.sh --imu
./run_visual_slam_3d.sh --pangolin-viewer
./run_visual_slam_3d.sh --dense-interval 4 --dense-voxel 0.03
./run_visual_slam_3d.sh --no-dense --no-viewer
```

- `--imu`：双目惯性模式。当前仓库的 `IMU.T_b_c1` 是单位阵占位值，完成相机-IMU 外参标定后再用于正式测量。
- `--opencv-viewer`：低延迟双目状态窗口，默认启用。
- `--pangolin-viewer`：三维稀疏地图窗口，适合检查结构，不适合最高帧率采集。
- `--dense` / `--no-dense`：启用/关闭异步稠密地图。
- `--dense-interval N`：每 N 个成功跟踪帧融合一次深度，默认 6。
- `--dense-width N`：稠密匹配宽度，默认 640。
- `--dense-stride N`：每隔 N 像素采样，默认 2。
- `--dense-voxel M`：体素边长（米），默认 0.05。
- `--dense-min-depth M` / `--dense-max-depth M`：有效深度范围。
- `--swap-eyes`：原始第二半幅作为 LEFT/Camera1，RERVISION 默认值。

完整实机说明、性能数据和状态解释见 [README_SLAM.md](README_SLAM.md)。

## 每次运行的输出

实时模式默认创建 `data/runtime/orbslam3/run_YYYYMMDD_HHMMSS/`：

- `dense_map.ply`：稠密灰度点云，使用 MeshLab 或 CloudCompare 查看。
- `map.ply`：过滤后的 ORB-SLAM3 稀疏地图点。
- `poses.csv`：逐帧位姿和跟踪状态。
- `tracking_health.csv`：逐帧特征数、地图点、IMU 样本数和耗时。
- `status.txt`：供状态脚本读取的最新健康信息。
- `CameraTrajectory.txt` / `KeyFrameTrajectory.txt`：EuRoC/TUM 工具可处理的轨迹。

离线模式额外生成 `summary.txt`，记录成功跟踪比例、点数和总耗时。

## 自动验收

在任何装有 Python、NumPy 和 OpenCV 的环境中执行：

```bash
# 普通 PC 可先安装工具依赖；Jetson 使用系统定制 OpenCV，不执行此行
python -m pip install -r requirements-tools.txt

python -m unittest discover -s tests -v
python tools/validate_project.py
python tools/validate_project.py --latest
python tools/validate_project.py --run data/runtime/orbslam3/run_YYYYMMDD_HHMMSS
```

校验器会检查项目文件、ORB 词典、两套 OpenCV YAML、鱼眼 JSON、轨迹有限值以及 PLY 顶点数量。`--latest` 会验收本机最新一次运行。

## 项目结构

```text
visual_slam/
├── configs/                        # 相机、双目和 IMU 参数
├── src/
│   ├── orbslam3_rervision.cpp      # 实时硬件适配器 + 稠密融合
│   └── orbslam3_stereo_dataset.cpp # 通用双目序列适配器
├── tools/
│   ├── prepare_stereo_dataset.py   # SBS 视频转数据集
│   └── validate_project.py         # 项目/运行结果验收
├── tests/                          # 无硬件自动测试
├── vendor/                         # ORB-SLAM3、相机 SDK、定制 OpenCV
├── data/runtime/                   # 运行产物（不纳入版本控制）
├── build_*.sh                      # 两个 C++ 入口的构建脚本
└── run_*.sh / start_*.sh           # 运行和运维入口
```

核心数据流：

```text
双目鱼眼帧 ─┬─> ORB-SLAM3 双目跟踪 ─> 局部建图 ─> 回环/图优化 ─> 位姿与稀疏地图
            └─> 异步鱼眼校正 ─> SGBM 深度 ─> 位姿融合 ─> 稠密体素 PLY
IMU（可选） ────────────────────────> 预积分 ────────────────────────┘
```

## 依赖与部署约束

目标平台为 Linux/Jetson，C++17。仓库使用定制 OpenCV 和 ORB-SLAM3 动态库；构建脚本默认寻找：

- Conda 环境：`/home/linaro/miniconda3/envs/fire-monitor`
- ORB-SLAM3：`vendor/ORB_SLAM3`
- OpenCV：`vendor/opencv_cuda/install`

部署到其他路径时可设置 `VISLAM_ENV_ROOT`、`VISLAM_ORB_ROOT`、`VISLAM_OPENCV_ROOT`，C++ 可执行文件自身不含固定项目路径。ORB 词典若只有 `ORBvoc.txt.tar.gz`，离线运行脚本会自动解压。

## 已知边界

- 仓库自带标定仅适用于当前这套双目相机及其固定镜头相对位置。
- IMU 外参仍是占位值；纯双目模式已具备真实尺度，正式使用 IMU 前必须完成时空联合标定。
- 稠密模块生成点云而非带纹理三角网格；网格重建属于后处理。
- 当前 Windows 工作区可运行数据准备、校验与测试；实时相机 SDK 和 ORB-SLAM3 二进制需要在目标 Linux/Jetson 上构建和运行。

编辑器索引配置位于 `.vscode/` 和 `pyrightconfig.json`。建议在 VS Code 中直接打开 `visual_slam` 目录；若打开它的上级目录，嵌套项目配置不会自动生效。Windows 本机没有 Eigen C++ 头文件时，ORB-SLAM3 的惯性类型可能仍显示单个外部依赖提示；通过 Remote SSH 打开 Jetson 项目后会使用 Conda 环境中的 Eigen 并完整解析。

ORB-SLAM3 及其派生链接程序遵循上游 GPLv3；`vendor/` 下其他组件分别遵循各自许可证。


## Git 仓库与第三方依赖

主项目同步至 https://github.com/garand-spec/visual_slam，替代原先的 yxt_visual_slam 演示程序。第三方版本、Jetson 定制补丁与首次克隆步骤见 [vendor/README.md](vendor/README.md)。设备上的数据、日志和编译产物保留在本地。
