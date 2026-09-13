#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <System.h>
#include "ImuTypes.h"
#include "scamlib.h"
#include "pose_telemetry.h"
#include "live_cloud.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <spawn.h>
#include <fcntl.h>
extern char** environ;

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

static void start_surface_reconstruction(const fs::path& project, const fs::path& run) {
    if (!fs::exists(run / "depth_frames/frames.csv")) return;
    const char* configured = std::getenv("VISLAM_MODEL_PYTHON");
    std::string python = configured ? configured : "python3";
    std::string script = (project / "tools/reconstruct_scene.py").string();
    std::string output = run.string();
    char* args[] = {python.data(), script.data(), const_cast<char*>("--run"), output.data(), nullptr};
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO,
        (run / "model_build.log").c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    posix_spawn_file_actions_adddup2(&actions, STDOUT_FILENO, STDERR_FILENO);
    posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
    posix_spawnattr_t attributes;
    posix_spawnattr_init(&attributes);
    posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETPGROUP);
    posix_spawnattr_setpgroup(&attributes, 0);
    pid_t pid = 0;
    const int result = posix_spawnp(&pid, python.c_str(), &actions, &attributes, args, environ);
    posix_spawn_file_actions_destroy(&actions);
    posix_spawnattr_destroy(&attributes);
    if (result == 0) std::cout << "surface reconstruction started pid=" << pid << '\n';
    else std::cerr << "surface reconstruction could not start: " << result << '\n';
}

struct Options {
    int device = 0;
    int format_index = 0;
    double seconds = 0.0;
    bool use_imu = false;
    bool pangolin_viewer = false;
    bool opencv_viewer = true;
    bool swap_eyes = true;
    bool settings_explicit = false;
    bool dense_mapping = true;
    int dense_interval = 6;
    int dense_width = 640;
    int dense_stride = 2;
    double dense_voxel = 0.05;
    double dense_min_depth = 0.3;
    double dense_max_depth = 12.0;
    std::string vocabulary;
    std::string settings;
    std::string output_root;
};

static fs::path project_root_from_executable(const char* argv0) {
    std::error_code error;
    fs::path executable = fs::canonical("/proc/self/exe", error);
    if (error) {
        executable = fs::absolute(argv0 ? fs::path(argv0) : fs::path("."), error);
    }
    // The production binaries live in <project>/bin. Falling back to cwd
    // keeps local developer builds usable as well.
    if (!executable.empty() && executable.has_parent_path() &&
        executable.parent_path().filename() == "bin") {
        return executable.parent_path().parent_path();
    }
    return fs::current_path();
}

static const char* tracking_state_name(int state) {
    switch (state) {
        case -1: return "SYSTEM_NOT_READY";
        case 0: return "NO_IMAGES_YET";
        case 1: return "INITIALIZING";
        case 2: return "OK";
        case 3: return "RECENTLY_LOST";
        case 4: return "LOST";
        case 5: return "OK_KLT";
        default: return "UNKNOWN";
    }
}

static bool tracking_is_ok(int state) {
    return state == 2 || state == 5;
}

struct FramePacket {
    std::vector<uint8_t> bytes;
    int width = 0;
    int height = 0;
    CamFormat format = FORMAT_UNKNOWN;
    bool nv12_luma_only = false;
    uint64_t timestamp_us = 0;
    std::array<sICM42688_XYZ_float, 11> imu{};
};

static std::atomic<bool> running{true};
static std::mutex frame_mutex;
static std::condition_variable frame_cv;
static std::shared_ptr<FramePacket> latest_frame;
static std::mutex display_mutex;
static std::condition_variable display_cv;
static cv::Mat latest_display_frame;
static std::atomic<bool> display_running{false};

// Accumulated IMU samples survive frame drops (latest-only queue) so the
// stereo-inertial preintegration never sees gaps between consumed frames.
static std::mutex imu_mutex;
static std::vector<ORB_SLAM3::IMU::Point> imu_buffer;
constexpr float kAccScale = 9.80665f / 1000.0f;  // SDK mg -> m/s^2
constexpr float kGyroScale = 3.14159265358979323846f / 180.0f;  // deg/s -> rad/s

static void stop_signal(int) {
    running.store(false);
    display_running.store(false);
}

static void display_loop() {
    try {
        cv::namedWindow("ORB-SLAM3 RERVISION", cv::WINDOW_NORMAL);
        cv::resizeWindow("ORB-SLAM3 RERVISION", 1280, 470);
        while (display_running.load()) {
            cv::Mat frame;
            {
                std::unique_lock<std::mutex> lock(display_mutex);
                display_cv.wait_for(lock, std::chrono::milliseconds(100), [] {
                    return !display_running.load() || !latest_display_frame.empty();
                });
                if (!display_running.load()) break;
                frame = std::move(latest_display_frame);
            }
            if (frame.empty()) continue;
            cv::imshow("ORB-SLAM3 RERVISION", frame);
            const int key = cv::waitKey(1) & 0xff;
            if (key == 27 || key == 'q') {
                running.store(false);
                frame_cv.notify_all();
                break;
            }
        }
        cv::destroyWindow("ORB-SLAM3 RERVISION");
        cv::waitKey(1);
    } catch (const cv::Exception& error) {
        std::cerr << "OpenCV viewer disabled: " << error.what() << "\n";
    }
    display_running.store(false);
}

static void frame_callback(const CamData* image, void*) {
    if (!image || !running.load() ||
        (image->format != FORMAT_NV12 && image->format != FORMAT_RGB24) ||
        !image->data || image->width < 2 || image->height < 2) {
        return;
    }

    const size_t required = image->format == FORMAT_NV12
        ? static_cast<size_t>(image->width) * image->height * 3 / 2
        : static_cast<size_t>(image->width) * image->height * 3;
    if (image->bufSize < required) return;

    auto packet = std::make_shared<FramePacket>();
    packet->width = image->width;
    packet->height = image->height;
    packet->format = image->format;
    packet->timestamp_us = image->endExpouseTime;
    // ORB-SLAM3 only consumes grayscale. For NV12, retain just the Y plane;
    // the UV plane is never read, so copying it only adds bandwidth and latency.
    const size_t bytes_to_copy = image->format == FORMAT_NV12
        ? static_cast<size_t>(image->width) * image->height
        : required;
    packet->nv12_luma_only = image->format == FORMAT_NV12;
    packet->bytes.assign(image->data, image->data + bytes_to_copy);
    for (size_t i = 0; i < packet->imu.size(); ++i) {
        packet->imu[i] = image->imu_data[i];
    }
    {
        std::lock_guard<std::mutex> lock(imu_mutex);
        for (const auto& sample : packet->imu) {
            imu_buffer.emplace_back(
                sample.fAccData_X * kAccScale,
                sample.fAccData_Y * kAccScale,
                sample.fAccData_Z * kAccScale,
                sample.fGyroData_X * kGyroScale,
                sample.fGyroData_Y * kGyroScale,
                sample.fGyroData_Z * kGyroScale,
                static_cast<double>(sample.uTime) * 1e-6);
        }
        if (imu_buffer.size() > 4000) {
            imu_buffer.erase(imu_buffer.begin(), imu_buffer.end() - 2000);
        }
    }

    {
        std::lock_guard<std::mutex> lock(frame_mutex);
        latest_frame = std::move(packet);
    }
    frame_cv.notify_one();
}

static bool parse_args(int argc, char** argv, Options& options,
                       const fs::path& project_root) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        auto next_string = [&](std::string& value) {
            if (++i >= argc) {
                std::cerr << "missing value for " << arg << "\n";
                return false;
            }
            value = argv[i];
            return true;
        };
        auto next_int = [&](int& value) {
            if (++i >= argc) {
                std::cerr << "missing value for " << arg << "\n";
                return false;
            }
            try {
                value = std::stoi(argv[i]);
            } catch (const std::exception&) {
                std::cerr << "invalid integer for " << arg << ": " << argv[i] << "\n";
                return false;
            }
            return true;
        };
        auto next_double = [&](double& value) {
            if (++i >= argc) {
                std::cerr << "missing value for " << arg << "\n";
                return false;
            }
            try {
                value = std::stod(argv[i]);
            } catch (const std::exception&) {
                std::cerr << "invalid number for " << arg << ": " << argv[i] << "\n";
                return false;
            }
            return true;
        };

        if (arg == "--vocabulary" && !next_string(options.vocabulary)) return false;
        if (arg == "--settings") {
            if (!next_string(options.settings)) return false;
            options.settings_explicit = true;
            continue;
        }
        if (arg == "--output-dir" && !next_string(options.output_root)) return false;
        if (arg == "--device" && !next_int(options.device)) return false;
        if (arg == "--format-index" && !next_int(options.format_index)) return false;
        if (arg == "--seconds" && !next_double(options.seconds)) return false;
        if (arg == "--imu") {
            options.use_imu = true;
            continue;
        }
        if (arg == "--swap-eyes") {
            options.swap_eyes = true;
            continue;
        }
        if (arg == "--no-swap-eyes") {
            options.swap_eyes = false;
            continue;
        }
        if (arg == "--dense") {
            options.dense_mapping = true;
            continue;
        }
        if (arg == "--no-dense") {
            options.dense_mapping = false;
            continue;
        }
        if (arg == "--dense-interval" && !next_int(options.dense_interval)) return false;
        if (arg == "--dense-width" && !next_int(options.dense_width)) return false;
        if (arg == "--dense-stride" && !next_int(options.dense_stride)) return false;
        if (arg == "--dense-voxel" && !next_double(options.dense_voxel)) return false;
        if (arg == "--dense-min-depth" && !next_double(options.dense_min_depth)) return false;
        if (arg == "--dense-max-depth" && !next_double(options.dense_max_depth)) return false;
        if (arg == "--no-viewer") {
            options.pangolin_viewer = false;
            options.opencv_viewer = false;
            continue;
        }
        if (arg == "--pangolin-viewer") {
            options.pangolin_viewer = true;
            options.opencv_viewer = false;
            continue;
        }
        if (arg == "--opencv-viewer") {
            options.opencv_viewer = true;
            continue;
        }
        if (arg == "--no-opencv-viewer") {
            options.opencv_viewer = false;
            continue;
        }
        if (arg == "--help" || arg == "-h") {
            std::cout
                << "Usage: run_orbslam3_rervision.sh [options]\n"
                << "  --vocabulary PATH\n"
                << "  --settings PATH\n"
                << "  --seconds N       0 means until Ctrl-C\n"
                << "  --imu             enable experimental stereo-inertial mode\n"
                << "  --swap-eyes       raw right half is Camera1/LEFT (default)\n"
                << "  --no-swap-eyes    preserve the original raw half order\n"
                << "  --dense           asynchronous dense stereo map (default)\n"
                << "  --no-dense        disable dense mapping\n"
                << "  --dense-interval N process one dense frame every N tracked frames\n"
                << "  --dense-width N   rectified dense stereo width (default 640)\n"
                << "  --dense-stride N  sample every N pixels into the voxel map\n"
                << "  --dense-voxel M   voxel size in metres (default 0.05)\n"
                << "  --dense-min-depth M / --dense-max-depth M\n"
                << "  --no-viewer       disable all viewers\n"
                << "  --pangolin-viewer show the 3D sparse map instead of the stereo window\n"
                << "  --opencv-viewer   enable the low-latency stereo/status window (default)\n"
                << "  --no-opencv-viewer disable the OpenCV status window\n"
                << "  --output-dir PATH\n";
            return false;
        }
    }
    if (options.vocabulary.empty()) {
        options.vocabulary = (project_root / "vendor" / "ORB_SLAM3" /
                              "Vocabulary" / "ORBvoc.txt").string();
    }
    if (!options.settings_explicit) {
        options.settings = (project_root / "configs" /
            (options.swap_eyes
                ? "orbslam3_rervision_stereo_inertial_swapped.yaml"
                : "orbslam3_rervision_stereo_inertial.yaml")).string();
    }
    if (options.output_root.empty()) {
        options.output_root =
            (project_root / "data" / "runtime" / "orbslam3").string();
    }
    return options.seconds >= 0.0 && options.dense_interval > 0 &&
           options.dense_width >= 320 && options.dense_stride > 0 &&
           options.dense_voxel > 0.0 && options.dense_min_depth > 0.0 &&
           options.dense_max_depth > options.dense_min_depth;
}

static size_t write_ply(const std::string& path,
                         const std::vector<ORB_SLAM3::MapPoint*>& map_points) {
    std::vector<Eigen::Vector3f> points;
    points.reserve(map_points.size());
    for (auto* point : map_points) {
        if (!point || point->isBad() || point->Observations() < 2 ||
            point->GetFoundRatio() < 0.25f) {
            continue;
        }
        const Eigen::Vector3f position = point->GetWorldPos();
        if (std::isfinite(position.x()) && std::isfinite(position.y()) &&
            std::isfinite(position.z())) {
            points.push_back(position);
        }
    }

    std::ofstream ply(path);
    if (!ply) return 0;
    ply << "ply\nformat ascii 1.0\n";
    ply << "element vertex " << points.size() << "\n";
    ply << "property float x\nproperty float y\nproperty float z\n";
    ply << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    ply << "end_header\n";
    for (const auto& p : points) {
        ply << p.x() << " " << p.y() << " " << p.z()
            << " 220 220 220\n";
    }
    return points.size();
}

struct VoxelKey {
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash {
    size_t operator()(const VoxelKey& key) const {
        size_t h = std::hash<int>{}(key.x);
        h ^= std::hash<int>{}(key.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<int>{}(key.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};

struct DenseVoxel {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double intensity = 0.0;
    uint32_t count = 0;
};

struct DenseFrame {
    cv::Mat left;
    cv::Mat right;
    Sophus::SE3f Tcw;
    int frame_id = 0;
    double timestamp = 0;
    unsigned long map_id = 0;
    unsigned long map_version = 0;
};

class DenseMapper {
public:
    DenseMapper(const Options& options, const fs::path& output)
        : options_(options), output_(output), live_cloud_(output / "live_dense_cloud.json") {}

    ~DenseMapper() {
        stop();
    }

    bool start() {
        try {
            cv::FileStorage settings(options_.settings, cv::FileStorage::READ);
            if (!settings.isOpened()) {
                std::cerr << "dense mapper: cannot open settings "
                          << options_.settings << "\n";
                return false;
            }

            const int source_width = static_cast<int>(settings["Camera.width"]);
            const int source_height = static_cast<int>(settings["Camera.height"]);
            if (source_width <= 0 || source_height <= 0) {
                std::cerr << "dense mapper: invalid calibrated image size\n";
                return false;
            }
            dense_size_ = cv::Size(
                options_.dense_width,
                std::max(1, static_cast<int>(std::lround(
                    static_cast<double>(options_.dense_width) * source_height /
                    source_width))));

            auto camera_matrix = [&](const char* prefix) {
                const std::string p(prefix);
                cv::Mat K = (cv::Mat_<double>(3, 3) <<
                    static_cast<double>(settings[p + ".fx"]), 0.0,
                    static_cast<double>(settings[p + ".cx"]),
                    0.0, static_cast<double>(settings[p + ".fy"]),
                    static_cast<double>(settings[p + ".cy"]),
                    0.0, 0.0, 1.0);
                const double sx = static_cast<double>(dense_size_.width) / source_width;
                const double sy = static_cast<double>(dense_size_.height) / source_height;
                K.at<double>(0, 0) *= sx;
                K.at<double>(0, 2) *= sx;
                K.at<double>(1, 1) *= sy;
                K.at<double>(1, 2) *= sy;
                return K;
            };
            auto distortion = [&](const char* prefix) {
                const std::string p(prefix);
                cv::Mat D = (cv::Mat_<double>(4, 1) <<
                    static_cast<double>(settings[p + ".k1"]),
                    static_cast<double>(settings[p + ".k2"]),
                    static_cast<double>(settings[p + ".k3"]),
                    static_cast<double>(settings[p + ".k4"]));
                return D;
            };

            const cv::Mat K1 = camera_matrix("Camera1");
            const cv::Mat K2 = camera_matrix("Camera2");
            const cv::Mat D1 = distortion("Camera1");
            const cv::Mat D2 = distortion("Camera2");
            cv::Mat T12;
            settings["Stereo.T_c1_c2"] >> T12;
            if (T12.rows != 4 || T12.cols != 4) {
                std::cerr << "dense mapper: Stereo.T_c1_c2 must be 4x4\n";
                return false;
            }
            T12.convertTo(T12, CV_64F);
            const cv::Mat R = T12(cv::Rect(0, 0, 3, 3)).clone();
            const cv::Mat t = T12(cv::Rect(3, 0, 1, 3)).clone();
            cv::Mat R1, R2, P1, P2, Q;
            cv::fisheye::stereoRectify(
                K1, D1, K2, D2, dense_size_, R, t,
                R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY,
                dense_size_, 0.0, 1.0);

            // The high-order fisheye fit can make OpenCV estimate a near-zero
            // pinhole focal length. Keep stereoRectify's epipolar rotations,
            // but use a deterministic ~90-degree rectified projection.
            const double rectified_tx = P2.at<double>(0, 3) / P2.at<double>(0, 0);
            const double rectified_focal = dense_size_.width * 0.5;
            const double rectified_cx = (dense_size_.width - 1) * 0.5;
            const double rectified_cy = (dense_size_.height - 1) * 0.5;
            P1 = cv::Mat::zeros(3, 4, CV_64F);
            P2 = cv::Mat::zeros(3, 4, CV_64F);
            for (cv::Mat* projection : {&P1, &P2}) {
                projection->at<double>(0, 0) = rectified_focal;
                projection->at<double>(1, 1) = rectified_focal;
                projection->at<double>(0, 2) = rectified_cx;
                projection->at<double>(1, 2) = rectified_cy;
                projection->at<double>(2, 2) = 1.0;
            }
            P2.at<double>(0, 3) = rectified_focal * rectified_tx;
            Q = (cv::Mat_<double>(4, 4) <<
                1.0, 0.0, 0.0, -rectified_cx,
                0.0, 1.0, 0.0, -rectified_cy,
                0.0, 0.0, 0.0, rectified_focal,
                0.0, 0.0, -1.0 / rectified_tx, 0.0);
            cv::fisheye::initUndistortRectifyMap(
                K1, D1, R1, P1, dense_size_, CV_32FC1, map1x_, map1y_);
            cv::fisheye::initUndistortRectifyMap(
                K2, D2, R2, P2, dense_size_, CV_32FC1, map2x_, map2y_);
            Q_ = Q;
            const cv::Mat R1t = R1.t();
            cv::Mat R1f;
            R1t.convertTo(R1f, CV_32F);
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    rectified_to_camera_(row, col) = R1f.at<float>(row, col);
                }
            }

            constexpr int num_disparities = 128;
            min_disparity_ = rectified_tx < 0.0 ? 0 : -num_disparities;
            stereo_ = cv::StereoSGBM::create(
                min_disparity_, num_disparities, 5,
                8 * 5 * 5, 32 * 5 * 5, 1, 31, 10, 80, 2,
                cv::StereoSGBM::MODE_SGBM_3WAY);
            right_min_disparity_ = -min_disparity_ - num_disparities + 1;
            stereo_right_ = cv::StereoSGBM::create(
                right_min_disparity_, num_disparities, 5,
                8 * 5 * 5, 32 * 5 * 5, 1, 31, 10, 80, 2,
                cv::StereoSGBM::MODE_SGBM_3WAY);
            fs::create_directories(output_ / "depth_frames");
            fs::create_directories(output_ / "dense_previews");
            depth_manifest_.open(output_ / "depth_frames/frames.csv");
            depth_manifest_ << "frame,timestamp_s,map_id,map_version,depth,image\n";
            std::ofstream calibration(output_ / "depth_frames/calibration.json");
            calibration << std::setprecision(15) << "{\"width\":" << dense_size_.width
                << ",\"height\":" << dense_size_.height << ",\"fx\":" << rectified_focal
                << ",\"fy\":" << rectified_focal << ",\"cx\":" << rectified_cx
                << ",\"cy\":" << rectified_cy << ",\"depth_scale\":1000,\"rectified_to_camera\":[";
            for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c)
                calibration << (r || c ? "," : "") << rectified_to_camera_(r,c);
            calibration << "]}\n";

            active_.store(true);
            worker_ = std::thread(&DenseMapper::worker_loop, this);
            std::cout << "dense mapper: " << dense_size_.width << "x"
                      << dense_size_.height << " min_disparity="
                      << min_disparity_ << " voxel=" << options_.dense_voxel
                      << "m interval=" << options_.dense_interval << "\n";
            return true;
        } catch (const cv::Exception& error) {
            std::cerr << "dense mapper initialization failed: "
                      << error.what() << "\n";
            return false;
        }
    }

    void submit(const cv::Mat& left, const cv::Mat& right,
                const Sophus::SE3f& Tcw, int frame_id, double timestamp,
                unsigned long map_id, unsigned long map_version) {
        if (!active_.load()) return;
        auto frame = std::make_shared<DenseFrame>();
        // cv::Mat reference counting keeps these immutable frame buffers alive;
        // replacing latest_ drops obsolete work instead of building a queue.
        frame->left = left;
        frame->right = right;
        frame->Tcw = Tcw;
        frame->frame_id = frame_id;
        frame->timestamp = timestamp;
        frame->map_id = map_id;
        frame->map_version = map_version;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_ = std::move(frame);
        }
        cv_.notify_one();
    }

    void stop() {
        if (!active_.exchange(false)) return;
        cv_.notify_all();
        if (worker_.joinable()) worker_.join();
        depth_manifest_.flush();
    }

    size_t point_count() const {
        return point_count_.load();
    }

    int fused_frames() const {
        return fused_frames_.load();
    }

    size_t write_ply(const std::string& path) const {
        std::ofstream ply(path);
        if (!ply) return 0;
        ply << "ply\nformat ascii 1.0\n";
        ply << "comment asynchronous fisheye stereo dense voxel map\n";
        ply << "element vertex " << voxels_.size() << "\n";
        ply << "property float x\nproperty float y\nproperty float z\n";
        ply << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
        ply << "end_header\n";
        for (const auto& entry : voxels_) {
            const DenseVoxel& voxel = entry.second;
            if (voxel.count == 0) continue;
            const double inv = 1.0 / voxel.count;
            const int color = std::clamp(
                static_cast<int>(std::lround(voxel.intensity * inv)), 0, 255);
            ply << voxel.x * inv << " " << voxel.y * inv << " "
                << voxel.z * inv << " " << color << " " << color << " "
                << color << "\n";
        }
        return voxels_.size();
    }

private:
    void worker_loop() {
        while (true) {
            std::shared_ptr<DenseFrame> frame;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [&] { return !active_.load() || latest_ != nullptr; });
                if (!active_.load()) break;
                frame = std::move(latest_);
            }
            if (!frame) continue;
            try {
                process(*frame);
            } catch (const cv::Exception& error) {
                std::cerr << "dense mapper frame " << frame->frame_id
                          << " failed: " << error.what() << "\n";
            }
        }
    }

    void process(const DenseFrame& frame) {
        if (have_epoch_ && frame.map_version != current_epoch_) {
            write_ply((output_ / "dense_previews" /
                ("map_" + std::to_string(current_map_) + "_epoch_" + std::to_string(current_epoch_) + ".ply")).string());
            voxels_.clear();
            point_count_.store(0);
        }
        have_epoch_ = true;
        current_epoch_ = frame.map_version;
        current_map_ = frame.map_id;
        cv::Mat left_small, right_small, left_rectified, right_rectified;
        cv::resize(frame.left, left_small, dense_size_, 0, 0, cv::INTER_AREA);
        cv::resize(frame.right, right_small, dense_size_, 0, 0, cv::INTER_AREA);
        cv::remap(left_small, left_rectified, map1x_, map1y_,
                  cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        cv::remap(right_small, right_rectified, map2x_, map2y_,
                  cv::INTER_LINEAR, cv::BORDER_CONSTANT);

        cv::Mat disparity16, disparity, points3d, right_disparity16, right_disparity;
        stereo_->compute(left_rectified, right_rectified, disparity16);
        stereo_right_->compute(right_rectified, left_rectified, right_disparity16);
        disparity16.convertTo(disparity, CV_32F, 1.0 / 16.0);
        right_disparity16.convertTo(right_disparity, CV_32F, 1.0 / 16.0);
        cv::reprojectImageTo3D(disparity, points3d, Q_, false, CV_32F);
        cv::Mat depth_mm = cv::Mat::zeros(dense_size_, CV_16UC1);

        if (fused_frames_.load() == 0) {
            double disparity_min = 0.0;
            double disparity_max = 0.0;
            cv::minMaxLoc(disparity, &disparity_min, &disparity_max);
            const cv::Mat valid_disparity =
                (disparity > min_disparity_ + 0.5f) &
                (disparity < min_disparity_ + 127.5f);
            std::cout << "dense diagnostic: nonzero_left="
                      << cv::countNonZero(left_rectified)
                      << " nonzero_right=" << cv::countNonZero(right_rectified)
                      << " valid_disparity=" << cv::countNonZero(valid_disparity)
                      << " disparity_range=[" << disparity_min << ","
                      << disparity_max << "]\n";
        }

        const Sophus::SE3f Twc = frame.Tcw.inverse();
        constexpr size_t max_voxels = 2000000;
        for (int y = 0; y < points3d.rows; ++y) {
            const auto* points = points3d.ptr<cv::Vec3f>(y);
            const auto* disparities = disparity.ptr<float>(y);
            const auto* intensities = left_rectified.ptr<uint8_t>(y);
            const auto* right_pixels = right_rectified.ptr<uint8_t>(y);
            const auto* right_disparities = right_disparity.ptr<float>(y);
            for (int x = 0; x < points3d.cols; ++x) {
                const float d = disparities[x];
                const int xr = std::isfinite(d) ? static_cast<int>(std::lround(x - d)) : -1;
                if (!std::isfinite(d) || d <= min_disparity_ + 0.5f ||
                    d >= min_disparity_ + 127.5f ||
                    xr < 0 || xr >= points3d.cols || intensities[x] < 3 || right_pixels[xr] < 3 ||
                    right_disparities[xr] < right_min_disparity_ ||
                    std::abs(d + right_disparities[xr]) > 1.5f) {
                    continue;
                }
                const cv::Vec3f& rectified = points[x];
                if (!std::isfinite(rectified[0]) || !std::isfinite(rectified[1]) ||
                    !std::isfinite(rectified[2])) {
                    continue;
                }
                const cv::Vec3f camera_cv = rectified_to_camera_ * rectified;
                const float depth = camera_cv[2];
                if (depth < options_.dense_min_depth ||
                    depth > options_.dense_max_depth) {
                    continue;
                }
                if (rectified[2] <= 0 || rectified[2] >= 65.535f) continue;
                depth_mm.at<uint16_t>(y,x) = static_cast<uint16_t>(std::lround(rectified[2] * 1000));
                if (y % options_.dense_stride || x % options_.dense_stride) continue;
                const Eigen::Vector3f camera(camera_cv[0], camera_cv[1], camera_cv[2]);
                const Eigen::Vector3f world = Twc * camera;
                if (!std::isfinite(world.x()) || !std::isfinite(world.y()) ||
                    !std::isfinite(world.z())) {
                    continue;
                }
                const VoxelKey key{
                    static_cast<int>(std::floor(world.x() / options_.dense_voxel)),
                    static_cast<int>(std::floor(world.y() / options_.dense_voxel)),
                    static_cast<int>(std::floor(world.z() / options_.dense_voxel))};
                auto found = voxels_.find(key);
                if (found == voxels_.end()) {
                    if (voxels_.size() >= max_voxels) continue;
                    found = voxels_.emplace(key, DenseVoxel{}).first;
                }
                DenseVoxel& voxel = found->second;
                voxel.x += world.x();
                voxel.y += world.y();
                voxel.z += world.z();
                voxel.intensity += intensities[x];
                ++voxel.count;
            }
        }
        if (live_cloud_.due()) {
            std::vector<std::array<float,3>> preview;
            const size_t stride=std::max<size_t>(1,(voxels_.size()+19999)/20000);
            size_t index=0;
            for(const auto& entry:voxels_) {
                if(index++ % stride)continue;
                const auto& v=entry.second;const float inv=1.0f/v.count;
                preview.push_back({v.x*inv,v.y*inv,v.z*inv});
            }
            live_cloud_.write(frame.frame_id,frame.timestamp,frame.map_id,frame.map_version,preview);
        }
        point_count_.store(voxels_.size());
        fused_frames_.fetch_add(1);
        // Keep reprojection evidence for final-pose surface reconstruction. Bounded
        // capture count prevents unbounded disk growth in unattended sessions.
        if (archived_frames_ < 2000 && cv::countNonZero(depth_mm) >= 1000) {
            const std::string stem = std::to_string(frame.frame_id);
            if (cv::imwrite((output_ / "depth_frames" / (stem + "_depth.png")).string(), depth_mm) &&
                cv::imwrite((output_ / "depth_frames" / (stem + "_gray.png")).string(), left_rectified)) {
                depth_manifest_ << frame.frame_id << ',' << std::setprecision(15) << frame.timestamp
                    << ',' << frame.map_id << ',' << frame.map_version << ','
                    << stem << "_depth.png," << stem << "_gray.png\n";
                depth_manifest_.flush();
                ++archived_frames_;
            }
        }
    }

    Options options_;
    fs::path output_;
    LiveCloudWriter live_cloud_;
    std::ofstream depth_manifest_;
    int archived_frames_ = 0;
    bool have_epoch_ = false;
    unsigned long current_epoch_ = 0, current_map_ = 0;
    cv::Size dense_size_;
    cv::Mat map1x_, map1y_, map2x_, map2y_, Q_;
    cv::Matx33f rectified_to_camera_ = cv::Matx33f::eye();
    cv::Ptr<cv::StereoSGBM> stereo_;
    cv::Ptr<cv::StereoSGBM> stereo_right_;
    int min_disparity_ = 0;
    int right_min_disparity_ = 0;
    std::atomic<bool> active_{false};
    std::atomic<size_t> point_count_{0};
    std::atomic<int> fused_frames_{0};
    std::thread worker_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::shared_ptr<DenseFrame> latest_;
    std::unordered_map<VoxelKey, DenseVoxel, VoxelKeyHash> voxels_;
};

static std::string make_output_dir(const std::string& root) {
    const auto now = std::chrono::system_clock::now();
    const auto tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&tt, &tm);
    std::ostringstream name;
    name << "run_" << std::put_time(&tm, "%Y%m%d_%H%M%S");
    const fs::path path = fs::path(root) / name.str();
    fs::create_directories(path);
    return path.string();
}

static bool read_calibrated_size(const std::string& settings_path,
                                 int& width, int& height) {
    try {
        cv::FileStorage settings(settings_path, cv::FileStorage::READ);
        if (!settings.isOpened()) return false;
        width = static_cast<int>(settings["Camera.width"]);
        height = static_cast<int>(settings["Camera.height"]);
        return width > 0 && height > 0;
    } catch (const cv::Exception&) {
        return false;
    }
}

static bool make_stereo_gray(const FramePacket& packet, cv::Mat& left, cv::Mat& right,
                             bool swap_eyes, int calibrated_width,
                             int calibrated_height, bool& resized_warned) {
    if (packet.width < 2 || packet.height < 2) return false;
    const int half_width = packet.width / 2;

    if (packet.format == FORMAT_NV12) {
        const size_t expected = static_cast<size_t>(packet.width) *
                                static_cast<size_t>(packet.height) *
                                (packet.nv12_luma_only ? 1 : 3) /
                                (packet.nv12_luma_only ? 1 : 2);
        if (packet.bytes.size() < expected) return false;
        // NV12 luma is already grayscale; avoid RGB conversion entirely.
        cv::Mat y(packet.height, packet.width, CV_8UC1,
                  const_cast<uint8_t*>(packet.bytes.data()));
        const cv::Mat first = y(cv::Rect(0, 0, half_width, packet.height));
        const cv::Mat second = y(cv::Rect(half_width, 0, half_width, packet.height));
        left = (swap_eyes ? second : first).clone();
        right = (swap_eyes ? first : second).clone();
    } else if (packet.format == FORMAT_RGB24) {
        const size_t expected = static_cast<size_t>(packet.width) *
                                static_cast<size_t>(packet.height) * 3;
        if (packet.bytes.size() < expected) return false;
        cv::Mat rgb(packet.height, packet.width, CV_8UC3,
                    const_cast<uint8_t*>(packet.bytes.data()));
        cv::Mat first_rgb = rgb(cv::Rect(0, 0, half_width, packet.height));
        cv::Mat second_rgb = rgb(cv::Rect(half_width, 0, half_width, packet.height));
        cv::Mat left_rgb = swap_eyes ? second_rgb : first_rgb;
        cv::Mat right_rgb = swap_eyes ? first_rgb : second_rgb;
        cv::cvtColor(left_rgb, left, cv::COLOR_RGB2GRAY);
        cv::cvtColor(right_rgb, right, cv::COLOR_RGB2GRAY);
    } else {
        return false;
    }

    // ORB-SLAM3 settings are calibrated for the resolution in the YAML.
    if (left.cols != calibrated_width || left.rows != calibrated_height) {
        if (!resized_warned) {
            std::cerr << "warning: input " << left.cols << "x" << left.rows
                      << " does not match calibrated " << calibrated_width << "x"
                      << calibrated_height << "; resizing (distortion model mismatch)\n";
            resized_warned = true;
        }
        cv::resize(left, left, cv::Size(calibrated_width, calibrated_height), 0, 0,
                   cv::INTER_AREA);
        cv::resize(right, right, cv::Size(calibrated_width, calibrated_height), 0, 0,
                   cv::INTER_AREA);
    }
    return !left.empty() && !right.empty();
}

static std::vector<ORB_SLAM3::IMU::Point> drain_imu_measurements(
    double previous_camera_time, double camera_time) {
    std::vector<ORB_SLAM3::IMU::Point> measurements;
    const double window_start = previous_camera_time >= 0.0
        ? previous_camera_time
        : camera_time - 0.5;
    const double window_end = camera_time + 0.005;
    std::lock_guard<std::mutex> lock(imu_mutex);
    const auto begin = std::lower_bound(
        imu_buffer.begin(), imu_buffer.end(), window_start,
        [](const ORB_SLAM3::IMU::Point& point, double value) {
            return point.t < value;
        });
    size_t kept = 0;
    for (auto it = begin; it != imu_buffer.end() && it->t <= window_end; ++it) {
        measurements.push_back(*it);
        ++kept;
    }
    imu_buffer.erase(begin, begin + static_cast<long>(kept));
    return measurements;
}

int main(int argc, char** argv) {
    Options options;
    const fs::path project_root = project_root_from_executable(argv[0]);
    try {
        if (!parse_args(argc, argv, options, project_root)) {
            std::cerr << "invalid command line (use --help)\n";
            return 1;
        }
    } catch (const std::exception& error) {
        std::cerr << "invalid command line: " << error.what() << "\n";
        return 1;
    }
    std::signal(SIGINT, stop_signal);
    std::signal(SIGTERM, stop_signal);

    if (!fs::exists(options.vocabulary)) {
        std::cerr << "vocabulary not found: " << options.vocabulary << "\n";
        return 2;
    }
    if (!fs::exists(options.settings)) {
        std::cerr << "settings not found: " << options.settings << "\n";
        return 2;
    }

    int calibrated_width = 0;
    int calibrated_height = 0;
    if (!read_calibrated_size(options.settings, calibrated_width, calibrated_height)) {
        std::cerr << "cannot read calibrated image size from " << options.settings << "\n";
        return 2;
    }
    bool resized_warned = false;

    if (!SCAM_Initialize()) {
        std::cerr << "SCAM_Initialize failed: "
                  << SCAM_GetErrorText(SCAM_GetLastError()) << "\n";
        return 3;
    }

    DeviceInfo devices[10]{};
    int device_count = 0;
    if (!SCAM_EnumDevices(devices, 10, &device_count) ||
        options.device < 0 || options.device >= device_count) {
        std::cerr << "camera enumeration failed\n";
        SCAM_Release();
        return 4;
    }

    FormatInfo formats[100]{};
    int format_count = 0;
    if (!SCAM_GetDeviceFormats(options.device, formats, 100, &format_count) ||
        options.format_index < 0 || options.format_index >= format_count) {
        std::cerr << "invalid camera format index\n";
        SCAM_Release();
        return 5;
    }

    std::cout << "camera format: " << formats[options.format_index].width << "x"
              << formats[options.format_index].height << " @ "
              << formats[options.format_index].fps << " FPS\n";

    if (!SCAM_SetDeviceFormat(options.device, options.format_index)) {
        std::cerr << "SCAM_SetDeviceFormat failed: "
                  << SCAM_GetErrorText(SCAM_GetLastError()) << "\n";
        SCAM_Release();
        return 6;
    }
    // Request NV12 so the Jetson nvvidconv path can provide luma directly.
    SCAM_SetImageFormat(options.device, FORMAT_NV12);
    if (!SCAM_OpenDevice(options.device, frame_callback, nullptr)) {
        std::cerr << "SCAM_OpenDevice failed: "
                  << SCAM_GetErrorText(SCAM_GetLastError()) << "\n";
        SCAM_Release();
        return 7;
    }

    int processed = 0;
    int tracking_ok = 0;
    int degraded_frames = 0;
    int lost_frames = 0;
    int loss_events = 0;
    int consecutive_lost = 0;
    int previous_state = -999;
    int imu_empty = 0;
    double previous_camera_time = -1.0;
    size_t map_points_cache = 0;
    int ok_frames_since_dense = 0;
    int consecutive_good = 0;
    auto start = Clock::now();
    std::thread display_thread;

    // Output runs are only created once the camera is confirmed working, so a
    // failed startup leaves no empty run_* directories behind.
    const std::string output_dir = make_output_dir(options.output_root);
    PoseTelemetry pose_telemetry(output_dir);
    LiveCloudWriter sparse_cloud(fs::path(output_dir) / "live_sparse_cloud.json");
    std::ofstream pose_csv(fs::path(output_dir) / "poses.csv");
    pose_csv << "frame,timestamp_s,state,map_points,tx,ty,tz,qx,qy,qz,qw,map_id,tracked_features,source_segment\n";
    std::ofstream health_csv(fs::path(output_dir) / "tracking_health.csv");
    health_csv << "frame,timestamp_s,state,state_name,detected_features,"
                  "tracked_features,map_points,"
                  "imu_samples,track_ms,consecutive_lost\n";
    fs::create_directories(options.output_root);
    {
        std::ofstream latest(fs::path(options.output_root) / "latest_run.txt");
        latest << output_dir << "\n";
    }

    std::cout << "ORB-SLAM3 RERVISION stereo-inertial\n"
              << "settings: " << options.settings << "\n"
              << "output: " << output_dir << "\n"
              << "mode: " << (options.use_imu ? "stereo-inertial" : "stereo visual") << "\n"
              << "eye order: " << (options.swap_eyes
                    ? "raw second half -> LEFT/Camera1 (swapped)"
                    : "raw first half -> LEFT/Camera1") << "\n"
              << "map export: filtered map points -> map.ply\n"
              << "dense map: " << (options.dense_mapping
                    ? "asynchronous stereo -> dense_map.ply"
                    : "disabled") << "\n"
              << "image path: NV12 luma via Jetson nvvidconv\n"
              << "ORB backend: CUDA ORB when available (CPU fallback enabled)\n";

    try {
        ORB_SLAM3::System slam(
            options.vocabulary, options.settings,
            options.use_imu ? ORB_SLAM3::System::IMU_STEREO
                            : ORB_SLAM3::System::STEREO,
            options.pangolin_viewer);
        DenseMapper dense_mapper(options, output_dir);
        const bool dense_active = options.dense_mapping && dense_mapper.start();
        if (options.opencv_viewer) {
            display_running.store(true);
            display_thread = std::thread(display_loop);
        }
        start = Clock::now();

        while (running.load() &&
               (options.seconds <= 0.0 ||
                std::chrono::duration<double>(Clock::now() - start).count() <
                    options.seconds)) {
            std::shared_ptr<FramePacket> packet;
            {
                std::unique_lock<std::mutex> lock(frame_mutex);
                frame_cv.wait_for(lock, std::chrono::milliseconds(200), [] {
                    return !running.load() || latest_frame != nullptr;
                });
                if (!running.load()) break;
                packet = std::move(latest_frame);
            }
            if (!packet) continue;

            cv::Mat left, right;
            if (!make_stereo_gray(*packet, left, right, options.swap_eyes,
                                  calibrated_width, calibrated_height,
                                  resized_warned)) continue;
            const double camera_time = static_cast<double>(packet->timestamp_us) * 1e-6;
            const auto imu = options.use_imu
                ? drain_imu_measurements(previous_camera_time, camera_time)
                : std::vector<ORB_SLAM3::IMU::Point>{};
            if (options.use_imu && processed > 0 && imu.empty()) ++imu_empty;

            const auto tracking_start = Clock::now();
            const Sophus::SE3f pose = slam.TrackStereo(
                left, right, camera_time, imu);
            const double tracking_ms = std::chrono::duration<double, std::milli>(
                Clock::now() - tracking_start).count();

            const int state = slam.GetTrackingState();
            // Existing map accessors are implemented in this ORB-SLAM3 build.
            // Record identity so the viewer never joins independent Atlas maps.
            const auto current_keyframes = slam.GetAllKeyFrames();
            const long long map_id = static_cast<long long>(slam.GetCurrentMapId());
            const unsigned long map_version = slam.GetMapVersion();
            // GetAllMapPoints copies the whole map; only refresh every 15 frames.
            if (processed % 15 == 0) {
                map_points_cache = slam.GetAllMapPoints().size();
            }
            const size_t map_points = map_points_cache;
            const auto tracked_keypoints = slam.GetTrackedKeyPointsUn();
            const auto tracked_map_points = slam.GetTrackedMapPoints();
            const size_t detected_features = tracked_keypoints.size();
            const size_t tracked_features = static_cast<size_t>(std::count_if(
                tracked_map_points.begin(), tracked_map_points.end(),
                [](ORB_SLAM3::MapPoint* point) {
                    return point != nullptr && !point->isBad();
                }));
            const auto t = pose.translation();
            const auto q = pose.unit_quaternion();
            const Sophus::SE3f world_pose = pose.inverse();
            const auto world_t = world_pose.translation();
            const auto world_q = world_pose.unit_quaternion();
            pose_telemetry.update(processed, camera_time, state, map_id, tracked_features,
                {world_t.x(), world_t.y(), world_t.z()},
                {world_q.x(), world_q.y(), world_q.z(), world_q.w()}, map_version);
            if(sparse_cloud.due()) {
                std::vector<std::array<float,3>> preview;
                if(tracking_is_ok(state)&&tracked_features>0) {
                    const auto points=slam.GetAllMapPoints();
                    const size_t stride=std::max<size_t>(1,(points.size()+5999)/6000);
                    for(size_t i=0;i<points.size();i+=stride) {
                        auto* point=points[i];if(!point||point->isBad())continue;
                        const auto xyz=point->GetWorldPos();
                        preview.push_back({xyz.x(),xyz.y(),xyz.z()});
                    }
                }
                sparse_cloud.write(processed,camera_time,map_id,map_version,preview);
            }
            pose_csv << processed << "," << std::setprecision(12) << camera_time
                     << "," << state << "," << map_points << ","
                     << t.x() << "," << t.y() << "," << t.z() << ","
                     << q.x() << "," << q.y() << "," << q.z() << "," << q.w()
                     << "," << map_id << "," << tracked_features << ","
                     << pose_telemetry.segment_id() << "\n";
            if (processed % 3 == 0) pose_csv.flush();

            const bool reliable_depth_pose = tracking_is_ok(state) && tracked_features >= 50 && current_keyframes.size() >= 2;
            consecutive_good = reliable_depth_pose ? consecutive_good + 1 : 0;
            if (!reliable_depth_pose) ok_frames_since_dense = 0;
            if (dense_active && consecutive_good >= 5) {
                ++ok_frames_since_dense;
                if (ok_frames_since_dense >= options.dense_interval) {
                    dense_mapper.submit(left, right, pose, processed, camera_time, map_id, map_version);
                    ok_frames_since_dense = 0;
                }
            }

            ++processed;
            if (tracking_is_ok(state)) {
                ++tracking_ok;
                consecutive_lost = 0;
            } else if (state == 3 || state == 4) {
                ++lost_frames;
                ++consecutive_lost;
                if (previous_state != 3 && previous_state != 4) ++loss_events;
            } else {
                ++degraded_frames;
                consecutive_lost = 0;
            }
            health_csv << processed - 1 << "," << std::setprecision(12)
                       << camera_time << "," << state << ","
                       << tracking_state_name(state) << "," << detected_features
                       << "," << tracked_features
                       << "," << map_points << "," << imu.size() << ","
                       << std::fixed << std::setprecision(3) << tracking_ms << ","
                       << consecutive_lost << "\n";
            previous_state = state;
            previous_camera_time = camera_time;

            if (processed == 1 || processed % 15 == 0) {
                std::ofstream status(fs::path(output_dir) / "status.txt");
                status << "state=" << tracking_state_name(state) << "\n"
                       << "frame=" << processed << "\n"
                       << "detected_features=" << detected_features << "\n"
                       << "tracked_features=" << tracked_features << "\n"
                       << "map_points=" << map_points << "\n"
                       << "dense_points=" << dense_mapper.point_count() << "\n"
                       << "dense_fused_frames=" << dense_mapper.fused_frames() << "\n"
                       << "track_ms=" << std::fixed << std::setprecision(1)
                       << tracking_ms << "\n"
                       << "consecutive_lost=" << consecutive_lost << "\n";
            }

            if (display_running.load() &&
                (processed == 1 || processed % 3 == 0)) {
                cv::Mat left_view, right_view, combined;
                cv::resize(left, left_view, cv::Size(640, 400), 0, 0, cv::INTER_AREA);
                cv::resize(right, right_view, cv::Size(640, 400), 0, 0, cv::INTER_AREA);
                cv::cvtColor(left_view, left_view, cv::COLOR_GRAY2BGR);
                cv::cvtColor(right_view, right_view, cv::COLOR_GRAY2BGR);
                const float sx = 640.0f / static_cast<float>(left.cols);
                const float sy = 400.0f / static_cast<float>(left.rows);
                const size_t drawable_count = std::min(
                    tracked_keypoints.size(), tracked_map_points.size());
                for (size_t i = 0; i < drawable_count; ++i) {
                    auto* map_point = tracked_map_points[i];
                    if (!map_point || map_point->isBad()) continue;
                    const auto& keypoint = tracked_keypoints[i];
                    const int x = static_cast<int>(keypoint.pt.x * sx);
                    const int y = static_cast<int>(keypoint.pt.y * sy);
                    if (x >= 0 && x < left_view.cols && y >= 0 && y < left_view.rows) {
                        cv::circle(left_view, cv::Point(x, y), 2,
                                   cv::Scalar(0, 220, 0), -1, cv::LINE_AA);
                    }
                }
                cv::hconcat(left_view, right_view, combined);
                cv::copyMakeBorder(combined, combined, 0, 70, 0, 0,
                                   cv::BORDER_CONSTANT, cv::Scalar(18, 18, 18));
                const cv::Scalar status_color = tracking_is_ok(state)
                    ? cv::Scalar(40, 220, 40)
                    : (state == 3 || state == 1)
                        ? cv::Scalar(0, 200, 255)
                        : cv::Scalar(40, 40, 255);
                cv::putText(combined, "LEFT", cv::Point(12, 28),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7,
                            cv::Scalar(255, 220, 80), 2, cv::LINE_AA);
                cv::putText(combined, "RIGHT", cv::Point(652, 28),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7,
                            cv::Scalar(255, 220, 80), 2, cv::LINE_AA);
                cv::putText(combined,
                            std::string("tracking=") + tracking_state_name(state) +
                            " detected=" + std::to_string(detected_features) +
                            " tracked=" + std::to_string(tracked_features) +
                            " map_points=" + std::to_string(map_points) +
                            " dense=" + std::to_string(dense_mapper.point_count()) +
                            " imu=" + std::to_string(imu.size()),
                            cv::Point(12, 430), cv::FONT_HERSHEY_SIMPLEX, 0.65,
                            status_color, 2, cv::LINE_AA);
                const std::string guidance = tracking_is_ok(state)
                    ? "OK: move slowly and keep mapped texture in both views"
                    : (state == 1)
                        ? "INITIALIZING: translate slowly; avoid rotating in place"
                        : "RECOVER: slow down and return to a known textured view";
                cv::putText(combined, guidance, cv::Point(12, 457),
                            cv::FONT_HERSHEY_SIMPLEX, 0.56,
                            cv::Scalar(235, 235, 235), 1, cv::LINE_AA);
                {
                    std::lock_guard<std::mutex> lock(display_mutex);
                    latest_display_frame = std::move(combined);
                }
                display_cv.notify_one();
            }

            if (processed == 1 || processed % 30 == 0) {
                std::cout << "frame=" << processed
                          << " state=" << tracking_state_name(state)
                          << " detected=" << detected_features
                          << " tracked=" << tracked_features
                          << " map_points=" << map_points
                          << " dense_points=" << dense_mapper.point_count()
                          << " dense_frames=" << dense_mapper.fused_frames()
                          << " imu=" << imu.size()
                          << " track_ms=" << std::fixed
                          << std::setprecision(1) << tracking_ms << "\n";
            }
        }

        dense_mapper.stop();
        size_t dense_exported = 0;
        if (dense_active) {
            dense_exported = dense_mapper.write_ply(
                (fs::path(output_dir) / "dense_map.ply").string());
            std::cout << "dense_frames=" << dense_mapper.fused_frames()
                      << " dense_points_exported=" << dense_exported << "\n";
        }
        slam.Shutdown();
        slam.SaveOptimizedFramePoses((fs::path(output_dir) / "optimized_poses.csv").string());
        const auto all_keyframes = slam.GetAllKeyFrames();
        if (!all_keyframes.empty()) {
            slam.SaveTrajectoryEuRoC(fs::path(output_dir) / "CameraTrajectory.txt");
            slam.SaveKeyFrameTrajectoryEuRoC(
                fs::path(output_dir) / "KeyFrameTrajectory.txt");
        } else {
            std::ofstream(fs::path(output_dir) / "CameraTrajectory.txt");
            std::ofstream(fs::path(output_dir) / "KeyFrameTrajectory.txt");
            std::cout << "no keyframes; trajectory files left empty\n";
        }
        auto all_map_points = slam.GetAllMapPoints();
        // Keep the useful map even if the last active map was a failed restart.
        for (auto* map : slam.GetAllMaps()) {
            if (!map || map->IsBad()) continue;
            const auto candidates = map->GetAllMapPoints();
            if (candidates.size() > all_map_points.size()) all_map_points = candidates;
        }
        const size_t exported_points = write_ply(
            fs::path(output_dir) / "map.ply", all_map_points);
        std::cout << "map_points_total=" << all_map_points.size()
                  << " map_points_exported=" << exported_points << "\n";
    } catch (const std::exception& error) {
        std::cerr << "ORB-SLAM3 exception: " << error.what() << "\n";
        running.store(false);
    }

    display_running.store(false);
    display_cv.notify_all();
    if (display_thread.joinable()) display_thread.join();

    pose_telemetry.finish();
    pose_csv.close();
    health_csv.close();
    SCAM_CloseDevice(options.device);
    SCAM_Release();

    const double elapsed = std::max(
        1e-6, std::chrono::duration<double>(Clock::now() - start).count());
    std::cout << "processed=" << processed
              << " tracking_ok=" << tracking_ok
              << " degraded=" << degraded_frames
              << " lost=" << lost_frames
              << " loss_events=" << loss_events
              << " no_imu_frames=" << imu_empty
              << " fps=" << std::fixed << std::setprecision(2)
              << processed / elapsed
              << " output=" << output_dir << "\n";
    const int exit_code = processed > 0 ? 0 : 8;
    if (processed > 0 && options.dense_mapping) start_surface_reconstruction(project_root, output_dir);
    std::cout.flush();
    std::cerr.flush();
    // Pangolin and GTK both register process-global teardown handlers. On this
    // Jetson build their static destruction order is undefined and can call a
    // released backend after all SLAM output has already been saved. All owned
    // resources are closed above, so bypass only those late global destructors.
    std::_Exit(exit_code);
}
