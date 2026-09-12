#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <System.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

struct Options {
    fs::path vocabulary;
    fs::path settings;
    fs::path left_dir;
    fs::path right_dir;
    fs::path timestamps;
    fs::path output_dir;
    double fps = 30.0;
    bool viewer = false;
    bool realtime = false;
};

static bool is_image(const fs::path& path) {
    std::string extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
    return extension == ".png" || extension == ".jpg" || extension == ".jpeg" ||
           extension == ".bmp" || extension == ".tif" || extension == ".tiff";
}

static std::vector<fs::path> list_images(const fs::path& directory) {
    std::vector<fs::path> images;
    if (!fs::is_directory(directory)) return images;
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.is_regular_file() && is_image(entry.path())) {
            images.push_back(entry.path());
        }
    }
    std::sort(images.begin(), images.end());
    return images;
}

static std::vector<double> read_timestamps(const fs::path& path) {
    std::vector<double> result;
    if (path.empty()) return result;
    std::ifstream input(path);
    std::string line;
    while (std::getline(input, line)) {
        const auto comment = line.find('#');
        if (comment != std::string::npos) line.resize(comment);
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream fields(line);
        double timestamp = 0.0;
        if (!(fields >> timestamp)) continue;
        // EuRoC stores nanoseconds (~1.7e18); microsecond stamps (~1.7e15)
        // also appear; ordinary manifests use seconds.
        if (timestamp > 1e16) timestamp *= 1e-9;
        else if (timestamp > 1e13) timestamp *= 1e-6;
        result.push_back(timestamp);
    }
    return result;
}

static bool parse_args(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string argument(argv[i]);
        auto next_path = [&](fs::path& value) {
            if (++i >= argc) return false;
            value = argv[i];
            return true;
        };
        if (argument == "--vocabulary" && !next_path(options.vocabulary)) return false;
        if (argument == "--settings" && !next_path(options.settings)) return false;
        if (argument == "--left" && !next_path(options.left_dir)) return false;
        if (argument == "--right" && !next_path(options.right_dir)) return false;
        if (argument == "--timestamps" && !next_path(options.timestamps)) return false;
        if (argument == "--output" && !next_path(options.output_dir)) return false;
        if (argument == "--fps") {
            if (++i >= argc) return false;
            options.fps = std::stod(argv[i]);
        } else if (argument == "--viewer") {
            options.viewer = true;
        } else if (argument == "--realtime") {
            options.realtime = true;
        } else if (argument == "--help" || argument == "-h") {
            std::cout
                << "ORB-SLAM3 generic stereo dataset runner\n\n"
                << "Required:\n"
                << "  --vocabulary PATH   ORBvoc.txt\n"
                << "  --settings PATH     ORB-SLAM3 stereo YAML\n"
                << "  --left DIR          left image directory\n"
                << "  --right DIR         right image directory\n"
                << "  --output DIR        output directory\n\n"
                << "Optional:\n"
                << "  --timestamps FILE   one timestamp per row (seconds or nanoseconds)\n"
                << "  --fps N             fallback rate when timestamps are absent (30)\n"
                << "  --viewer            enable Pangolin viewer\n"
                << "  --realtime          sleep to dataset time instead of running flat-out\n";
            return false;
        }
    }
    return !options.vocabulary.empty() && !options.settings.empty() &&
           !options.left_dir.empty() && !options.right_dir.empty() &&
           !options.output_dir.empty() && options.fps > 0.0;
}

static const char* state_name(int state) {
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

static size_t export_map(const fs::path& path,
                         const std::vector<ORB_SLAM3::MapPoint*>& map_points) {
    std::vector<Eigen::Vector3f> points;
    for (auto* point : map_points) {
        if (!point || point->isBad() || point->Observations() < 2 ||
            point->GetFoundRatio() < 0.25f) continue;
        const Eigen::Vector3f p = point->GetWorldPos();
        if (p.allFinite()) points.push_back(p);
    }
    std::ofstream ply(path);
    ply << "ply\nformat ascii 1.0\n"
        << "comment ORB-SLAM3 sparse map\n"
        << "element vertex " << points.size() << "\n"
        << "property float x\nproperty float y\nproperty float z\n"
        << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        << "end_header\n";
    for (const auto& p : points) {
        ply << p.x() << ' ' << p.y() << ' ' << p.z() << " 230 230 230\n";
    }
    return points.size();
}

int main(int argc, char** argv) {
    Options options;
    try {
        if (!parse_args(argc, argv, options)) return 1;
    } catch (const std::exception& error) {
        std::cerr << "invalid command line: " << error.what() << '\n';
        return 1;
    }
    if (!fs::is_regular_file(options.vocabulary) ||
        !fs::is_regular_file(options.settings)) {
        std::cerr << "vocabulary or settings file does not exist\n";
        return 2;
    }
    const auto left_images = list_images(options.left_dir);
    const auto right_images = list_images(options.right_dir);
    if (left_images.empty() || left_images.size() != right_images.size()) {
        std::cerr << "left/right image counts must be equal and non-zero (left="
                  << left_images.size() << ", right=" << right_images.size() << ")\n";
        return 3;
    }
    auto timestamps = read_timestamps(options.timestamps);
    if (!timestamps.empty() && timestamps.size() != left_images.size()) {
        std::cerr << "timestamp count does not match image count\n";
        return 4;
    }
    if (timestamps.empty()) {
        timestamps.resize(left_images.size());
        for (size_t i = 0; i < timestamps.size(); ++i) {
            timestamps[i] = static_cast<double>(i) / options.fps;
        }
    }
    fs::create_directories(options.output_dir);
    std::ofstream poses(options.output_dir / "poses.csv");
    poses << "frame,timestamp_s,state,state_name,map_points,tx,ty,tz,qx,qy,qz,qw,track_ms\n";

    std::cout << "images=" << left_images.size() << " settings=" << options.settings
              << " output=" << options.output_dir << '\n';
    ORB_SLAM3::System slam(options.vocabulary.string(), options.settings.string(),
                           ORB_SLAM3::System::STEREO, options.viewer);
    size_t ok_frames = 0;
    const auto run_start = Clock::now();
    const double timestamp_origin = timestamps.front();

    for (size_t i = 0; i < left_images.size(); ++i) {
        cv::Mat left = cv::imread(left_images[i].string(), cv::IMREAD_GRAYSCALE);
        cv::Mat right = cv::imread(right_images[i].string(), cv::IMREAD_GRAYSCALE);
        if (left.empty() || right.empty() || left.size() != right.size()) {
            std::cerr << "invalid stereo pair at frame " << i << '\n';
            slam.Shutdown();
            return 5;
        }
        const auto tracking_start = Clock::now();
        const Sophus::SE3f pose = slam.TrackStereo(left, right, timestamps[i]);
        const double tracking_ms = std::chrono::duration<double, std::milli>(
            Clock::now() - tracking_start).count();
        const int state = slam.GetTrackingState();
        const size_t map_points = slam.GetAllMapPoints().size();
        const auto t = pose.translation();
        const auto q = pose.unit_quaternion();
        poses << i << ',' << std::setprecision(12) << timestamps[i] << ','
              << state << ',' << state_name(state) << ',' << map_points << ','
              << t.x() << ',' << t.y() << ',' << t.z() << ','
              << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ','
              << std::fixed << std::setprecision(3) << tracking_ms << '\n';
        if (state == 2 || state == 5) ++ok_frames;
        if (i == 0 || (i + 1) % 100 == 0) {
            std::cout << "frame=" << (i + 1) << " state=" << state_name(state)
                      << " map_points=" << map_points << " track_ms="
                      << std::fixed << std::setprecision(1) << tracking_ms << '\n';
        }
        if (options.realtime && i + 1 < timestamps.size()) {
            const double target = timestamps[i + 1] - timestamp_origin;
            const double elapsed = std::chrono::duration<double>(Clock::now() - run_start).count();
            if (target > elapsed) {
                std::this_thread::sleep_for(std::chrono::duration<double>(target - elapsed));
            }
        }
    }

    slam.Shutdown();
    if (!slam.GetAllKeyFrames().empty()) {
        slam.SaveTrajectoryEuRoC(options.output_dir / "CameraTrajectory.txt");
        slam.SaveKeyFrameTrajectoryEuRoC(options.output_dir / "KeyFrameTrajectory.txt");
    } else {
        std::ofstream(options.output_dir / "CameraTrajectory.txt");
        std::ofstream(options.output_dir / "KeyFrameTrajectory.txt");
    }
    const size_t exported = export_map(options.output_dir / "map.ply",
                                       slam.GetAllMapPoints());
    const double elapsed = std::chrono::duration<double>(Clock::now() - run_start).count();
    std::ofstream summary(options.output_dir / "summary.txt");
    summary << "frames=" << left_images.size() << '\n'
            << "tracking_ok_frames=" << ok_frames << '\n'
            << "tracking_ok_ratio=" << std::setprecision(6)
            << static_cast<double>(ok_frames) / left_images.size() << '\n'
            << "map_points_exported=" << exported << '\n'
            << "elapsed_s=" << elapsed << '\n';
    std::cout << "complete frames=" << left_images.size() << " ok=" << ok_frames
              << " map_points=" << exported << " elapsed_s=" << elapsed << '\n';
    return ok_frames > 0 ? 0 : 6;
}
