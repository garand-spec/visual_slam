#include "StereoInitGate.h"
#include <cassert>
#include <limits>
#include <vector>
struct Point {
    float depth;
    bool allFinite() const { return std::isfinite(depth); }
    float z() const { return depth; }
};
struct Frame {
    int Nleft = -1;
    std::vector<int> mvLeftToRightMatch;
    std::vector<Point> mvStereo3Dpoints;
    std::vector<float> mvDepth;
};
int main() {
    Frame f;
    f.mvDepth.assign(1200, -1);
    assert(ORB_SLAM3::CountValidStereoSeeds(f) == 0);
    f.mvDepth.assign(50, 2);
    f.mvDepth.push_back(std::numeric_limits<float>::quiet_NaN());
    f.mvDepth.push_back(0);
    assert(ORB_SLAM3::CountValidStereoSeeds(f) == 50);
    f.Nleft = 100;
    f.mvLeftToRightMatch.assign(100, 0);
    f.mvStereo3Dpoints.assign(50, Point{2});
    assert(ORB_SLAM3::CountValidStereoSeeds(f) == 50);
    f.mvLeftToRightMatch[0] = -1;
    f.mvStereo3Dpoints[1].depth = -2;
    f.mvStereo3Dpoints[2].depth = std::numeric_limits<float>::infinity();
    assert(ORB_SLAM3::CountValidStereoSeeds(f) == 47);
}
