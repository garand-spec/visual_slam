#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

// Depth and its original pose form one durable association independent of Atlas resets.
class DepthArchive {
public:
    explicit DepthArchive(const std::filesystem::path& run, uintmax_t reserve=1073741824ULL)
        :run_(run),reserve_(reserve) {}
    void start() {
        std::filesystem::create_directories(run_/"depth_frames");
        manifest_.open(run_/"depth_frames/frames.csv");
        if(!manifest_)throw std::runtime_error("cannot open depth manifest");
        manifest_ << "frame,timestamp_s,map_id,map_version,depth,image,tx,ty,tz,qx,qy,qz,qw,state,tracked_features,source_segment\n";
        manifest_.flush();status("recording");
    }
    bool write(int frame,double timestamp,long long map,unsigned long epoch,long long segment,
               size_t support,const std::array<float,3>& t,const std::array<float,4>& q,
               const cv::Mat& depth,const cv::Mat& gray) {
        bool valid=std::isfinite(timestamp)&&map>=0&&support>0;
        for(float v:t)valid=valid&&std::isfinite(v);
        for(float v:q)valid=valid&&std::isfinite(v);
        if(!valid){++failures_;status("invalid_pose");return false;}
        try {
            const auto required=depth.total()*depth.elemSize()+gray.total()*gray.elemSize()+1048576ULL;
            const auto available=std::filesystem::space(run_).available;
            if(available<reserve_||available-reserve_<required){++failures_;status("low_disk");return false;}
            const auto stem=std::to_string(frame);
            const auto depth_name=stem+"_depth.png",gray_name=stem+"_gray.png";
            const auto directory=run_/"depth_frames";
            if(!cv::imwrite((directory/(stem+"_depth.tmp.png")).string(),depth)||
               !cv::imwrite((directory/(stem+"_gray.tmp.png")).string(),gray))throw std::runtime_error("depth image write failed");
            std::filesystem::rename(directory/(stem+"_depth.tmp.png"),directory/depth_name);
            std::filesystem::rename(directory/(stem+"_gray.tmp.png"),directory/gray_name);
            manifest_ << frame << ',' << std::setprecision(17) << timestamp << ',' << map << ',' << epoch
                      << ',' << depth_name << ',' << gray_name;
            for(float v:t)manifest_ << ',' << v;
            for(float v:q)manifest_ << ',' << v;
            manifest_ << ",2," << support << ',' << segment << '\n';manifest_.flush();
            if(!manifest_)throw std::runtime_error("depth manifest write failed");
            ++count_;last_frame_=frame;status("recording");return true;
        }catch(const std::exception&){++failures_;status("io_error");return false;}
    }
    void finish(){manifest_.flush();status(state_=="recording"?"finished":state_);}
private:
    void status(const std::string& state) {
        if(state!=state_&&state!="recording"&&state!="finished")
            std::cerr << "depth archive: " << state << "; capture continues, failed frames=" << failures_ << '\n';
        state_=state;
        const auto path=run_/"depth_archive_status.json";const auto temp=path.string()+".tmp";
        std::ofstream out(temp);
        out << "{\"state\":\"" << state << "\",\"archived_frames\":" << count_ << ",\"failed_frames\":" << failures_
            << ",\"last_frame\":" << last_frame_ << ",\"disk_reserve_bytes\":" << reserve_ << "}\n";
        out.close();if(out){std::error_code error;std::filesystem::rename(temp,path,error);}
    }
    std::filesystem::path run_;
    uintmax_t reserve_;
    std::ofstream manifest_;
    size_t count_=0,failures_=0;
    int last_frame_=-1;
    std::string state_="recording";
};
