#include "depth_archive.h"
#include <cassert>
#include <limits>
int main(int argc,char**argv) {
    assert(argc==2);
    std::filesystem::path root(argv[1]);
    DepthArchive archive(root/"normal",0);archive.start();
    cv::Mat depth(8,8,CV_16UC1,cv::Scalar(1000)),gray(8,8,CV_8UC1,cv::Scalar(180));
    for(int i=0;i<2005;++i)
        assert(archive.write(i,i*.05,0,i<2000?1:2,i<2000?0:1,100,{1,2,3},{0,0,0,1},depth,gray));
    archive.finish();
    assert(std::filesystem::exists(root/"normal/depth_frames/2004_depth.png"));
    std::ifstream manifest(root/"normal/depth_frames/frames.csv");
    std::string line,last;size_t count=0;
    while(std::getline(manifest,line)){last=line;++count;}
    assert(count==2006);
    assert(last.find("2004,")==0);
    assert(std::abs(std::stod(last.substr(5))-100.2)<1e-9);
    assert(last.find(",0,2,2004_depth.png,2004_gray.png,1,2,3,0,0,0,1,2,100,1")!=std::string::npos);
    DepthArchive blocked(root/"full",std::numeric_limits<uintmax_t>::max());blocked.start();
    assert(!blocked.write(0,0,0,1,0,100,{0,0,0},{0,0,0,1},depth,gray));blocked.finish();
    std::ifstream status(root/"full/depth_archive_status.json");std::getline(status,line);
    assert(line.find("low_disk")!=std::string::npos);
}
