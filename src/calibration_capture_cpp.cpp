#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "scamlib.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

struct Options {
    int device = 0;
    int format_index = 0;
    int board_cols = 11;
    int board_rows = 8;
    int detect_width = 1280;
    double detect_interval = 0.25;
    double seconds = 0.0;
    std::string output_root;
};

struct Frame {
    std::vector<uint8_t> bytes;
    int width = 0;
    int height = 0;
    CamFormat format = FORMAT_UNKNOWN;
};

struct Detection {
    bool left_found = false;
    bool right_found = false;
    std::vector<cv::Point2f> left_corners;
    std::vector<cv::Point2f> right_corners;
    Clock::time_point updated_at{};
};

// Do not pass cv::Mat objects between threads. Pass owned bytes instead and
// create non-owning Mats inside the detector while the request stays alive.
struct DetectionRequest {
    std::vector<uint8_t> left_bytes;
    std::vector<uint8_t> right_bytes;
    int width = 0;
    int height = 0;
};

static std::atomic<bool> running{true};
static std::mutex frame_mutex;
static std::condition_variable frame_cv;
static std::shared_ptr<Frame> latest_frame;

static void stop_signal(int) {
    running.store(false);
    frame_cv.notify_all();
}

static void frame_callback(const CamData* image, void*) {
    if (!image || !running.load() || image->format != FORMAT_RGB24 ||
        !image->data || image->bufSize <= 0) return;
    auto frame = std::make_shared<Frame>();
    frame->bytes.assign(image->data, image->data + image->bufSize);
    frame->width = image->width;
    frame->height = image->height;
    frame->format = image->format;
    {
        std::lock_guard<std::mutex> lock(frame_mutex);
        latest_frame = std::move(frame);
    }
    frame_cv.notify_one();
}

static bool parse_args(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        auto next_int = [&](int& v) {
            if (++i >= argc) return false;
            v = std::stoi(argv[i]); return true;
        };
        auto next_double = [&](double& v) {
            if (++i >= argc) return false;
            v = std::stod(argv[i]); return true;
        };
        auto next_string = [&](std::string& v) {
            if (++i >= argc) return false;
            v = argv[i]; return true;
        };
        if (arg == "--device" && !next_int(o.device)) return false;
        else if (arg == "--format-index" && !next_int(o.format_index)) return false;
        else if (arg == "--board-cols" && !next_int(o.board_cols)) return false;
        else if (arg == "--board-rows" && !next_int(o.board_rows)) return false;
        else if (arg == "--detect-width" && !next_int(o.detect_width)) return false;
        else if (arg == "--detect-interval" && !next_double(o.detect_interval)) return false;
        else if (arg == "--seconds" && !next_double(o.seconds)) return false;
        else if (arg == "--output-dir" && !next_string(o.output_root)) return false;
        else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: run_calibration_capture_cpp.sh [options]\n"
                      << "  --board-cols N      inner corners horizontally (default 11)\n"
                      << "  --board-rows N      inner corners vertically (default 8)\n"
                      << "  --seconds N         stop after N seconds; 0 means until q\n"
                      << "  --detect-width N    detector width (default 1280)\n";
            return false;
        }
    }
    return o.board_cols >= 2 && o.board_rows >= 2;
}

static bool to_stereo_bgr(const Frame& frame, cv::Mat& left, cv::Mat& right) {
    if (frame.format != FORMAT_RGB24 || frame.width < 2 || frame.height < 2) return false;
    const size_t need = static_cast<size_t>(frame.width) * frame.height * 3;
    if (frame.bytes.size() < need) return false;
    cv::Mat rgb(frame.height, frame.width, CV_8UC3,
                const_cast<uint8_t*>(frame.bytes.data()));
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    const int half = frame.width / 2;
    left = bgr(cv::Rect(0, 0, half, frame.height)).clone();
    right = bgr(cv::Rect(half, 0, half, frame.height)).clone();
    return true;
}

static bool detect_board(const cv::Mat& image, cv::Size pattern, int detect_width,
                         std::vector<cv::Point2f>& corners) {
    if (image.empty() || image.cols <= 0 || image.rows <= 0 || detect_width <= 0) {
        return false;
    }
    const double scale = std::min(1.0, static_cast<double>(detect_width) / image.cols);
    cv::Mat small;
    if (scale < 1.0) cv::resize(image, small, cv::Size(), scale, scale, cv::INTER_AREA);
    else small = image;
    cv::Mat gray;
    cv::cvtColor(small, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Mat> candidates{gray};
    cv::Mat equalized;
    cv::equalizeHist(gray, equalized);
    candidates.push_back(equalized);

    const int sb_flags = cv::CALIB_CB_NORMALIZE_IMAGE |
                         cv::CALIB_CB_EXHAUSTIVE |
                         cv::CALIB_CB_ACCURACY;
    for (const cv::Mat& candidate : candidates) {
        std::vector<cv::Point2f> found;
        if (cv::findChessboardCornersSB(candidate, pattern, found, sb_flags)) {
            if (scale < 1.0) {
                for (auto& p : found) {
                    p.x = static_cast<float>(p.x / scale);
                    p.y = static_cast<float>(p.y / scale);
                }
            }
            corners = std::move(found);
            return true;
        }
    }

    std::vector<cv::Point2f> classic;
    const int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    if (cv::findChessboardCorners(gray, pattern, classic, flags)) {
        cv::cornerSubPix(gray, classic, cv::Size(7, 7), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT,
                                          30, 0.01));
        if (scale < 1.0) {
            for (auto& p : classic) {
                p.x = static_cast<float>(p.x / scale);
                p.y = static_cast<float>(p.y / scale);
            }
        }
        corners = std::move(classic);
        return true;
    }
    return false;
}

static cv::Mat draw_view(const cv::Mat& image, bool found,
                         const std::vector<cv::Point2f>& corners,
                         cv::Size pattern, const std::string& name) {
    cv::Mat view;
    cv::resize(image, view, cv::Size(640, 400), 0, 0, cv::INTER_AREA);
    if (found && !corners.empty()) {
        std::vector<cv::Point2f> scaled = corners;
        const float sx = 640.0f / image.cols;
        const float sy = 400.0f / image.rows;
        for (auto& p : scaled) { p.x *= sx; p.y *= sy; }
        cv::drawChessboardCorners(view, pattern, scaled, true);
    }
    cv::putText(view, name + (found ? ": FOUND" : ": searching"),
                cv::Point(12, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                found ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 100, 255),
                2, cv::LINE_AA);
    return view;
}

static std::string make_output_dir(const std::string& root) {
    const auto now = std::chrono::system_clock::now();
    const auto tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&tt, &tm);
    std::ostringstream name;
    name << "run_" << std::put_time(&tm, "%Y%m%d_%H%M%S");
    fs::path path = fs::path(root) / name.str();
    fs::create_directories(path);
    return path.string();
}

int main(int argc, char** argv) {
    Options o;
    try {
        if (!parse_args(argc, argv, o)) return 1;
    } catch (const std::exception& error) {
        std::cerr << "invalid command line: " << error.what() << "\n";
        return 1;
    }
    if (o.output_root.empty()) {
        std::error_code error;
        fs::path executable = fs::canonical("/proc/self/exe", error);
        const fs::path project_root =
            !error && executable.parent_path().filename() == "bin"
                ? executable.parent_path().parent_path()
                : fs::current_path();
        o.output_root = (project_root / "data" / "calibration" / "raw").string();
    }
    std::signal(SIGINT, stop_signal);
    std::signal(SIGTERM, stop_signal);

    const cv::Size pattern(o.board_cols, o.board_rows);
    const std::string output_dir = make_output_dir(o.output_root);
    std::cout << "C++ calibration capture\n"
              << "board inner corners: " << o.board_cols << " x " << o.board_rows << "\n"
              << "output: " << output_dir << "\n";

    if (!SCAM_Initialize()) {
        std::cerr << "SCAM_Initialize failed: "
                  << SCAM_GetErrorText(SCAM_GetLastError()) << "\n";
        return 2;
    }
    DeviceInfo devices[10]{};
    int device_count = 0;
    if (!SCAM_EnumDevices(devices, 10, &device_count) ||
        o.device < 0 || o.device >= device_count) {
        std::cerr << "device enumeration failed or device unavailable\n";
        SCAM_Release();
        return 3;
    }
    FormatInfo formats[100]{};
    int format_count = 0;
    if (!SCAM_GetDeviceFormats(o.device, formats, 100, &format_count) ||
        o.format_index < 0 || o.format_index >= format_count) {
        std::cerr << "invalid format index\n";
        SCAM_Release();
        return 4;
    }
    std::cout << "device=" << o.device << " format="
              << formats[o.format_index].width << "x" << formats[o.format_index].height
              << " @ " << formats[o.format_index].fps << " fps\n";
    if (!SCAM_SetDeviceFormat(o.device, o.format_index)) {
        std::cerr << "SCAM_SetDeviceFormat failed: "
                  << SCAM_GetErrorText(SCAM_GetLastError()) << "\n";
        SCAM_Release();
        return 5;
    }
    SCAM_SetImageFormat(o.device, FORMAT_RGB24);
    if (!SCAM_OpenDevice(o.device, frame_callback, nullptr)) {
        std::cerr << "SCAM_OpenDevice failed: "
                  << SCAM_GetErrorText(SCAM_GetLastError()) << "\n";
        SCAM_Release();
        return 6;
    }

    std::shared_ptr<DetectionRequest> pending_request;
    std::mutex detect_mutex;
    std::condition_variable detect_cv;
    Detection detection;
    std::mutex detection_mutex;
    std::atomic<bool> detect_running{true};

    std::thread detector([&] {
        while (detect_running.load()) {
            std::shared_ptr<DetectionRequest> request;
            {
                std::unique_lock<std::mutex> lock(detect_mutex);
                detect_cv.wait_for(lock, std::chrono::milliseconds(200), [&] {
                    return pending_request != nullptr || !detect_running.load();
                });
                if (!detect_running.load()) break;
                request.swap(pending_request);
            }
            std::vector<cv::Point2f> lc, rc;
            if (!request || request->width <= 0 || request->height <= 0) continue;
            const size_t image_bytes = static_cast<size_t>(request->width) *
                                       static_cast<size_t>(request->height) * 3;
            if (request->left_bytes.size() < image_bytes ||
                request->right_bytes.size() < image_bytes) continue;
            cv::Mat left_image(request->height, request->width, CV_8UC3,
                               request->left_bytes.data());
            cv::Mat right_image(request->height, request->width, CV_8UC3,
                                request->right_bytes.data());
            bool lf = detect_board(left_image, pattern, o.detect_width, lc);
            bool rf = detect_board(right_image, pattern, o.detect_width, rc);
            std::lock_guard<std::mutex> lock(detection_mutex);
            detection.left_found = lf;
            detection.right_found = rf;
            detection.left_corners = std::move(lc);
            detection.right_corners = std::move(rc);
            detection.updated_at = Clock::now();
        }
    });

    bool window_ok = true;
    try {
        cv::namedWindow("RERVISION C++ Fisheye Calibration", cv::WINDOW_NORMAL);
        cv::resizeWindow("RERVISION C++ Fisheye Calibration", 1280, 460);
    } catch (const cv::Exception& e) {
        window_ok = false;
        std::cerr << "OpenCV window failed: " << e.what() << "\n";
    }
    std::cout << "Press c when both sides show FOUND to save; press q or ESC to quit.\n";

    const auto start = Clock::now();
    auto last_submit = start - std::chrono::seconds(1);
    int processed = 0;
    int saved = 0;

    while (running.load() &&
           (o.seconds <= 0.0 ||
            std::chrono::duration<double>(Clock::now() - start).count() < o.seconds)) {
        std::shared_ptr<Frame> frame;
        {
            std::unique_lock<std::mutex> lock(frame_mutex);
            frame_cv.wait_for(lock, std::chrono::milliseconds(200), [] {
                return !running.load() || latest_frame != nullptr;
            });
            if (!running.load()) break;
            frame = std::move(latest_frame);
        }
        if (!frame) continue;

        cv::Mat left, right;
        if (!to_stereo_bgr(*frame, left, right)) continue;
        processed++;
        const auto now = Clock::now();

        if (std::chrono::duration<double>(now - last_submit).count() >=
            std::max(0.05, o.detect_interval)) {
            auto request = std::make_shared<DetectionRequest>();
            request->width = left.cols;
            request->height = left.rows;
            const size_t image_bytes = left.total() * left.elemSize();
            request->left_bytes.assign(left.data, left.data + image_bytes);
            request->right_bytes.assign(right.data, right.data + image_bytes);
            {
                std::lock_guard<std::mutex> lock(detect_mutex);
                pending_request = std::move(request);
            }
            last_submit = now;
            detect_cv.notify_one();
        }

        Detection current;
        {
            std::lock_guard<std::mutex> lock(detection_mutex);
            current = detection;
        }
        const bool fresh = current.updated_at.time_since_epoch().count() != 0 &&
            std::chrono::duration<double>(now - current.updated_at).count() <=
                std::max(1.0, o.detect_interval * 4.0);
        const bool lf = fresh && current.left_found;
        const bool rf = fresh && current.right_found;

        if (window_ok) {
            cv::Mat lv = draw_view(left, lf, current.left_corners, pattern, "LEFT");
            cv::Mat rv = draw_view(right, rf, current.right_corners, pattern, "RIGHT");
            cv::Mat combined;
            cv::hconcat(lv, rv, combined);
            cv::putText(combined, "saved: " + std::to_string(saved) +
                        " | c=capture q=quit", cv::Point(12, 450),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0),
                        2, cv::LINE_AA);
            cv::imshow("RERVISION C++ Fisheye Calibration", combined);
            const int key = cv::waitKey(1) & 0xFF;
            if (key == 27 || key == 'q') running.store(false);
            if (key == 'c') {
                if (!lf || !rf) {
                    std::cout << "Both sides must show FOUND before capture.\n";
                } else {
                    std::ostringstream index;
                    index << std::setw(4) << std::setfill('0') << saved;
                    cv::imwrite(output_dir + "/left_" + index.str() + ".png", left);
                    cv::imwrite(output_dir + "/right_" + index.str() + ".png", right);
                    ++saved;
                    std::cout << "saved pair " << saved << "\n";
                }
            }
        }
    }

    detect_running.store(false);
    detect_cv.notify_all();
    if (detector.joinable()) detector.join();
    if (window_ok) cv::destroyAllWindows();
    SCAM_CloseDevice(o.device);
    SCAM_Release();

    const double elapsed = std::max(1e-6,
        std::chrono::duration<double>(Clock::now() - start).count());
    std::cout << "processed_frames=" << processed
              << " processing_fps=" << processed / elapsed
              << " saved_pairs=" << saved
              << " output_dir=" << output_dir << "\n";
    return 0;
}
