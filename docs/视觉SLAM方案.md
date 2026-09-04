# 视觉 SLAM 系统技术方案

> 平台：NVIDIA Jetson AGX Thor（Ubuntu 24.04 / aarch64）
> 传感器：RERVISION 双目鱼眼相机（含随帧 IMU）
> 主线算法：ORB-SLAM3（双目 + 可选惯性）+ 异步稠密建图
> 文档版本：v1.0（2026-09-04）

---

## 1. 目标与范围

构建一套可在 Jetson AGX Thor 上**实时运行、真尺度输出**的视觉 SLAM 系统：

1. **实时定位**：60fps 双目输入下持续输出 6DoF 位姿，真实米制尺度（无需里程计）。
2. **稀疏建图**：关键帧地图 + 回环检测 + 全局 BA，长时运行不漂移。
3. **稠密建图**：异步生成融合体素点云，退出时导出 PLY，可转网格。
4. **视觉惯性（VIO）可选**：接入随帧 IMU，弱纹理/快速运动场景下保持跟踪。
5. **离线可复现**：任意录制视频可转数据集回放，用于调参与回归测试。
6. **工程可运维**：前后台启停、优雅退出、状态诊断、日志与轨迹持久化。

非目标（当前阶段不做）：多机协同 SLAM、语义建图、在线地图服务化。

## 2. 硬件平台

| 项目 | 规格 |
|---|---|
| 计算平台 | NVIDIA Jetson AGX Thor Developer Kit |
| GPU | Blackwell 架构，2048 CUDA 核心，CUDA 13.2 / driver 595.78 |
| CPU | 14 核 Arm Neoverse V3AE（最高 2.6GHz） |
| 内存 | 128GB LPDDR5x 统一内存（CPU/GPU 共享） |
| 存储 | NVMe 936GB |
| 系统 | Ubuntu 24.04 LTS（aarch64） |
| 相机 | RERVISION 双目鱼眼 + IMU（GStreamer JPEG 流，60fps/目） |

Thor 的统一内存架构对 SLAM 非常有利：图像、IMU 缓冲、点云全部无需 PCIe 拷贝即可被 GPU 与 CPU 同时访问；GPU 算力（约 275 TFLOPS FP32）足以覆盖特征提取、立体匹配等全部 GPU 环节。

## 3. 传感器配置

- **双目鱼眼**：Kannala-Brandt（4 阶）畸变模型，单目 1920×1200，基线约 **0.1066 m**（来自标定向量 T_c1_c2）。
- **IMU**：相机随帧输出，SDK 提供 mg / deg/s 原始值，运行层适配器负责换算 SI 单位；标称频率 600Hz。
- **已知问题**：
  - 原始拼接画面左右半幅与旧程序标签相反 → 运行层默认 `--swap-eyes`，同时自动加载成对交换后的内参与逆外参（只换画面不换标定会导致系统性错误）。
  - 相机走 USB 2.0 JPEG 压缩流，需 `SCAM_GST_DECODER=jpegdec` 与 `libgst_compat.so`（LD_PRELOAD）做设备节点兼容。

> 备注：当前板端还挂有一只普通 USB 相机（HZXC，video0/1），用于 `yxt_visual_slam` 光流 Demo；正式 SLAM 链路以 RERVISION 双目为主。

## 4. 技术路线与算法选型

### 4.1 候选方案对比

| 方案 | 尺度 | 双目 | 鱼眼 | IMU | Jetson 适配 | 维护成本 | 结论 |
|---|---|---|---|---|---|---|---|
| **ORB-SLAM3** | 真尺度（双目） | 原生 | 需扩展（已做 KB8） | 有 | 已验证 | 中 | **选用** |
| VINS-Mono/Stereo | 单目无尺度需标定 | 需改 | 支持 | 强 | 可行 | 中 | 备选 |
| OpenVINS | 单目无尺度 | 需改 | 有限 | 强 | 可行 | 中 | 备选 |
| DSO / SVO | 单目无尺度 | 有限 | 有限 | 无 | 弱 | 高 | 不选 |
| Kimera (VIO+MSCKF) | 真尺度（双目） | 原生 | 需改 | 强 | 重 | 高 | 不选 |

**决策**：以 **ORB-SLAM3 双目前端**为主干，IMU 作为可选增强。理由：

1. 双目直接给出真尺度，省去单目尺度观测链路；
2. 回环检测（DBoW2 + 几何验证）+ 全局 BA 是长期建图不漂移的关键，ORB-SLAM3 工程成熟度最高；
3. 团队已完成 ORB-SLAM3 的 KannalaBrandt8 鱼眼扩展、RERVISION 相机适配与 GPU 构建，边际成本最低；
4. IMU 模式已预留接口，待外参标定后启用，可平滑升级到 VIO。

### 4.2 总体结构：稀疏定位 + 异步稠密

```
        ┌────────────────────────────────────────────────────────┐
        │                    主线程（实时）                       │
 相机帧 ─▶ 去畸变/缩放 ─▶ ORB-SLAM3 Tracking ─▶ 位姿输出          │
 (60fps)   (GPU)          (稀疏前端+回环+BA)   (真尺度 6DoF)       │
        │                     │ 关键帧/位姿队列                   │
        │                     ▼                                   │
        │        ┌─────────────────────────────┐                  │
        │        │  DenseMapper 线程（异步）    │                  │
        │        │ 鱼眼校正 → SGBM 深度 →      │                  │
        │        │ 位姿反投影 → 体素去重融合    │                  │
        │        └─────────────────────────────┘                  │
        │                     │                                   │
        │        ┌────────────▼────────────┐                      │
        │        │ 显示线程（默认 OpenCV    │                      │
        │        │ 状态窗口；可选 Pangolin  │                      │
        │        │ 稀疏地图 3D 视图）        │                      │
        │        └─────────────────────────┘                      │
        └────────────────────────────────────────────────────────┘
 优雅退出：SIGINT → 停止取帧 → 排空队列 → 导出轨迹 txt + dense_map.ply
```

设计原则：**定位前端只做逐点特征匹配**保证实时性；逐像素深度计算放到独立线程，与 SLAM 共享统一内存，互不阻塞。

## 5. 相机与传感器标定

| 项目 | 模型/方法 | 状态 |
|---|---|---|
| 单目内参 | Kannala-Brandt 4 阶，棋盘格采集 | ✅ 已标定（2026-08-30，`data/calibration/raw`） |
| 双目外参 | 立体标定 + 基线 0.1066m | ✅ 已标定（含交换版） |
| 相机-IMU 外参 T_b_c1 | 静态/动态 T 型标定 + 噪声辨识 | ❌ **当前为单位阵占位，VIO 正式启用前必须完成** |
| IMU 噪声参数 | Allman 方法 / 温漂实验 | ⚠️ 当前用经验值（gyro 0.001, acc 0.01） |

标定流程（已有工具链）：
`capture_calibration.py / calibration_capture_cpp`（采集）→ `calibrate_fisheye.py`（求解 JSON）→ 生成 ORB-SLAM3 YAML（`configs/`）。
IMU 外参计划：固定 IMU-相机刚体 → 多姿态静态采集 + 匀速转动采集 → 联合最小二乘求解时间偏移与旋转平移 → 用 `configs` 中 `IMU.T_b_c1` 落地。

## 6. 稠密建图流水线（DenseMapper）

1. **校正**：鱼眼 → 针孔透视投影（默认宽 640，16:10），GPU 执行；
2. **深度**：SGBM 立体匹配（GPU），有效深度 0.3–12 m；
3. **反投影**：按当前位姿将深度图反投影为相机系点云；
4. **融合**：体素哈希去重（默认 0.05 m，室内精细 0.03 m），像素步长 2；
5. **输出**：运行中增量维护，优雅退出导出 `dense_map.ply`。

调参旋钮（`run_visual_slam_3d.sh`）：`--dense-interval`（默认每 6 帧融合一次）、`--dense-width`、`--dense-stride`、`--dense-voxel`、`--dense-min/max-depth`。

## 7. 性能指标与实测基线

| 指标 | 目标 | 实测/现状 |
|---|---|---|
| 取帧率 | 60 fps | RERVISION 60fps/目 JPEG 流 |
| 定位帧率（OpenCV 窗口） | ≥ 20 fps | **约 21 fps**（含显示） |
| 定位帧率（Pangolin 3D 窗口） | ≥ 10 fps | 约 2 fps（X11/GTK 锁争用，待优化） |
| 稠密融合 | 不阻塞定位 | 异步线程，interval=6 下 CPU 占用可控 |
| 定位精度（室内 10m 回环） | 回环后轨迹误差 < 0.5% 里程 | 待 M3 评测 |
| 真尺度 | 米制，无尺度跳变 | 双目光学基线保证 |

优化方向（M4）：
- ORB 提取全程 GPU（`ORBextractor` 已走 CUDA OpenCV 构建）；
- SGBM 替换/并行化为 `cuda::cudawarping` 或半全局 GPU 实现；
- Pangolin 与地图线程争用：改为共享内存帧缓冲 + 独立渲染进程，或默认 OpenCV 窗口 + 周期性离屏快照。

## 8. 软件部署

- **环境**：conda 环境 `fire-monitor`（python 3.12、numpy、cmake、eigen 3.4、boost、pangolin-opengl），定义于 `environment.yml`。
- **构建**：
  - `./build_orbslam3_rervision.sh` → `bin/orbslam3_rervision`（实时主程序，链接 GPU OpenCV `vendor/opencv_cuda/install`）；
  - `./build_stereo_dataset.sh` → 离线数据集回放器。
- **运行**：
  ```bash
  ./run_visual_slam_3d.sh                # 默认：双目 + 稠密 + OpenCV 状态窗口
  ./run_visual_slam_3d.sh --imu          # 双目惯性模式（需先完成 IMU 外参）
  ./run_visual_slam_3d.sh --pangolin-viewer
  ./run_visual_slam_3d.sh --no-viewer --seconds 30
  ```
- **运维**：`start_slam.sh` / `status_slam.sh` / `stop_slam.sh`；日志 `data/runtime/orbslam3/slam.log`；停止必须走优雅退出以完成轨迹与点云落盘。
- **离线回放**：`tools/prepare_stereo_dataset.py`（视频→左右序列+时间戳）→ `run_stereo_dataset.sh`，用于无相机复现与回归测试。

## 9. 评测方案（M3）

1. **数据集**：实机录制 3–5 段室内视频（含回环路径），`prepare_stereo_dataset.py` 转标准序列；
2. **真值**：激光雷达里程计或结构光跟踪（如有条件）对齐时间戳；无真值时采用回环闭合度 + 已知距离参照物（门框/卷尺）评估；
3. **指标**：ATE（轨迹误差）、RPE（单步旋转/平移误差）、回环成功率、跟踪丢失时长占比、点云重投影一致性；
4. **回归**：每次算法/标定变更后用固定数据集回放，输出指标 JSON 归档。

## 10. 里程碑计划

| 里程碑 | 内容 | 出口准则 | 状态 |
|---|---|---|---|
| **M1 双目前端** | 鱼眼标定、ORB-SLAM3 双目跟踪、稠密 PLY、运维脚本、离线回放 | 实机持续 TRACKING OK，退出导出轨迹+PLY | ✅ 已完成 |
| **M2 VIO 落地** | 相机-IMU 外参 + 时间偏移标定、IMU 噪声辨识、`--imu` 正式启用 | 快速运动/短暂遮挡（≤2s）下不丢跟踪 | ⬜ 当前重点 |
| **M3 精度评测** | 评测数据集、ATE/RPE 基线、回环调参 | 10m 室内回环 ATE < 0.5% 里程 | ⬜ |
| **M4 性能优化** | GPU 深度、显示线程重构、长时运行稳定性（>4h 无泄漏） | 定位 ≥ 30fps；Pangolin ≥ 10fps | ⬜ |
| **M5 产品化** | 开机自启、远程状态监控、地图持久化/增量建图、打包交付 | 一键部署新板端 < 30min | ⬜ |

## 11. 风险与对策

| 风险 | 影响 | 对策 |
|---|---|---|
| IMU 外参未标定（当前单位阵占位） | VIO 模式正式测量不可用 | M2 优先；在此之前对外只承诺纯双目模式 |
| 长时运行点云/关键帧膨胀 | 内存增长、BA 变慢 | 体素上限 + 关键帧剔除策略（ORB-SLAM3 已有 PlaceborneCulling 类机制，需调参）；定期重启兜底 |
| USB 2.0 JPEG 流丢帧/抖动 | 时间戳漂移、跟踪退化 | 校验帧率稳定性；必要时升级 USB 3.0 链路或 GMSL 相机 |
| Pangolin/X11 争用 | 3D 查看卡顿 | 默认 OpenCV 状态窗口；Pangolin 仅作离线检查 |
| ORB-SLAM3 GPL 许可 | 商用分发受限 | 内部研究使用无碍；若商用需评估授权或替换（VINS/自研前端） |
| 弱纹理/低光照场景 | 跟踪丢失 | IMU 模式 + 回环恢复；采集时保证纹理与光照（已有状态提示：INITIALIZING/WEAK/LOST） |
| 板端电源模式波动 | 性能抖动 | 固定 `nvpmodel` 高性能模式并写入开机配置 |

## 12. 现有工程资产清单（服务器 `/home/linaro/projects/visual_slam`）

| 资产 | 说明 |
|---|---|
| `src/orbslam3_rervision.cpp`（1075 行） | 实时主程序：相机适配 + SLAM + DenseMapper + 显示线程 |
| `src/orbslam3_stereo_dataset.cpp` | 离线双目数据集回放器 |
| `src/calibration_capture_cpp.cpp` / `capture_calibration.py` / `calibrate_fisheye.py` / `camera_preview.py` | 标定与预览工具链 |
| `vendor/ORB_SLAM3` | 含 KannalaBrandt8 鱼眼扩展的 ORB-SLAM3（1.8G，含 ORB 词袋） |
| `vendor/opencv_cuda` | GPU 版 OpenCV 4.12（含 cuda 模块，1.1G） |
| `vendor/rervision_single_imu` | RERVISION 相机/IMU 适配库 + `libgst_compat.so` |
| `configs/*.yaml` | 双目标定（含 swap 版）+ IMU 占位参数 |
| `build_*.sh` / `run_*.sh` / `start/status/stop_slam.sh` | 构建与运维脚本 |
| `tools/prepare_stereo_dataset.py` | 视频→同步立体序列 |
| `data/calibration/raw` | 2026-08-30 标定采集数据（60 帧组） |

---

### 附：与 GitHub 仓库的关系

- `garand-spec/visual_slam`（本仓库）：`yxt_visual_slam` 光流 Demo + 本文档，作为方案与原型入口；
- 服务器 `visual_slam`：完整 SLAM 工程（含 vendor 大体积二进制，不入 git）；
- 后续建议：将本仓库 `docs/` 作为方案与实验记录主阵地，工程代码在服务器构建、二进制产物与 vendor 不入库。
