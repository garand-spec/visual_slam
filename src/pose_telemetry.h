#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include <unistd.h>

// One atomic, read-only snapshot for the LAN viewer. No networking on the SLAM thread.
class PoseTelemetry {
public:
    explicit PoseTelemetry(const std::filesystem::path& run_dir)
        : path_(run_dir / "viewer_state.json") { write("running"); }

    void update(int frame, double timestamp, int state, long long map_id, size_t tracked_features,
                const std::array<double, 3>& position,
                const std::array<double, 4>& quaternion, unsigned long map_version = 0) {
        bool valid = (state == 2 || state == 5) && tracked_features > 0 && map_id >= 0 && std::isfinite(timestamp);
        for (double v : position) valid = valid && std::isfinite(v);
        for (double v : quaternion) valid = valid && std::isfinite(v);
        if (valid && (!previous_valid_ || map_id != map_id_ || map_version != map_version_)) ++segment_id_;
        previous_valid_ = valid;
        map_version_ = map_version;
        tracked_features_ = tracked_features;
        frame_ = frame;
        timestamp_ = timestamp;
        state_ = state;
        map_id_ = map_id;
        position_ = position;
        quaternion_ = quaternion;
        const auto now = std::chrono::steady_clock::now();
        if (now - last_write_ >= std::chrono::milliseconds(100)) {
            write("running");
            last_write_ = now;
        }
    }

    void finish() { write("finished"); }
    long long segment_id() const { return segment_id_; }

private:
    void write(const char* lifecycle) const {
        bool finite = std::isfinite(timestamp_);
        for (double v : position_) finite = finite && std::isfinite(v);
        for (double v : quaternion_) finite = finite && std::isfinite(v);
        const bool valid = finite && previous_valid_;
        const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const auto temp = path_.string() + ".tmp";
        std::ofstream out(temp);
        if (!out) return;
        out << std::setprecision(15)
            << "{\"version\":1,\"lifecycle\":\"" << lifecycle
            << "\",\"pid\":" << getpid() << ",\"unix_ms\":" << millis
            << ",\"frame\":" << frame_ << ",\"timestamp_s\":" << (finite ? timestamp_ : 0)
            << ",\"state\":" << state_ << ",\"map_id\":" << map_id_
            << ",\"map_version\":" << map_version_
            << ",\"source_segment\":" << segment_id_ << ",\"tracked_features\":" << tracked_features_
            << ",\"valid\":" << (valid ? "true" : "false")
            << ",\"coordinate\":\"Twc; metres; xyzw\",\"p\":[";
        for (size_t i = 0; i < 3; ++i) out << (i ? "," : "") << (finite ? position_[i] : 0);
        out << "],\"q\":[";
        for (size_t i = 0; i < 4; ++i) out << (i ? "," : "") << (finite ? quaternion_[i] : (i == 3 ? 1 : 0));
        out << "]}\n";
        out.close();
        if (!out) return;
        std::error_code error;
        std::filesystem::rename(temp, path_, error);
    }

    std::filesystem::path path_;
    std::chrono::steady_clock::time_point last_write_{};
    int frame_ = -1;
    int state_ = -1;
    long long map_id_ = -1;
    long long segment_id_ = -1;
    unsigned long map_version_ = 0;
    size_t tracked_features_ = 0;
    bool previous_valid_ = false;
    double timestamp_ = 0;
    std::array<double, 3> position_{0, 0, 0};
    std::array<double, 4> quaternion_{0, 0, 0, 1};
};
