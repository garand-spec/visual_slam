#pragma once
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <vector>

// Each producer owns a separate file and writer; readers see only complete snapshots.
class LiveCloudWriter {
public:
    explicit LiveCloudWriter(const std::filesystem::path& path) : path_(path) {}
    bool due() const { return std::chrono::steady_clock::now()-last_ >= std::chrono::milliseconds(500); }
    void write(int frame, double timestamp, long long map, unsigned long epoch,
               const std::vector<std::array<float,3>>& points) {
        if (!std::isfinite(timestamp)) return;
        const auto temporary=path_.string()+".tmp";
        std::ofstream out(temporary);
        const auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        out << std::setprecision(12) << "{\"frame\":" << frame << ",\"timestamp_s\":" << timestamp
            << ",\"map_id\":" << map << ",\"map_version\":" << epoch << ",\"unix_ms\":" << ms
            << ",\"points\":[";
        bool first=true;size_t count=0;
        for(const auto& p:points) {
            if(!std::isfinite(p[0])||!std::isfinite(p[1])||!std::isfinite(p[2]))continue;
            if(count++>=20000)break;
            out << (first?"":",") << '[' << p[0] << ',' << p[1] << ',' << p[2] << ']';first=false;
        }
        out << "]}\n";out.close();
        if(out){std::error_code error;std::filesystem::rename(temporary,path_,error);}
        last_=std::chrono::steady_clock::now();
    }
private:
    std::filesystem::path path_;
    std::chrono::steady_clock::time_point last_{};
};
