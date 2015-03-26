#include "LinemodWrap.h"

using namespace std;
using namespace pcl;

LinemodWrap t;

int main() {
    string filename = "/home/gmanfred/devel/essential/ros/linemod_ros/views/fusee/view_0.pcd";
    PointCloudType::Ptr cloud (new PointCloudType());
    t.loadCloud(filename, *cloud);

    //t.trainTemplate(cloud);

    return 0;
}

