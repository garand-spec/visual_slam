#include "live_cloud.h"
#include "pose_telemetry.h"
#include <cassert>
#include <limits>
int main(int argc,char** argv) {
    assert(argc==2);
    std::filesystem::path root(argv[1]);
    LiveCloudWriter writer(root/"cloud.json");
    assert(writer.due());
    std::vector<std::array<float,3>> points(20002,{1,2,3});
    points[0][0]=std::numeric_limits<float>::quiet_NaN();
    writer.write(12,1.25,2,7,points);
    assert(!writer.due());
    assert(!std::filesystem::exists(root/"cloud.json.tmp"));
    PoseTelemetry telemetry(root);
    telemetry.update(12,1.25,2,2,50,{0,0,0},{0,0,0,1},7);
    auto segment=telemetry.segment_id();
    telemetry.update(13,1.30,2,2,50,{0,0,0},{0,0,0,1},8);
    assert(telemetry.segment_id()==segment+1);
    telemetry.finish();
}
