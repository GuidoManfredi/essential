#include <iostream>

#include "FilesManager.h"
#include "POM.h"
//#include "POD.h"

using namespace std;
using namespace boost::filesystem;
using namespace cv;

Pipeline2D pipeline2d;
POM pom;

void testModeling () {
    string intrinsics = "/home/gmanfred/.ros/camera_info/webcam_gilgamesh_opencv.yml";
    //pom.loadIntrinsic(intrinsics);

    FilesManager fm;
    string base = "/home/gmanfred/devel/datasets/my_objects/pom/lentilles";
    vector<Mat> images = fm.getImages (base);
    vector< vector<Point2f> > corners = fm.getCorners (base + "corners.txt");
    Point3f dims = fm.getDimensions (base + "dimensions.txt");
    Object obj = pom.model (CYL, images, corners, dims);
}

int main()
{
    testModeling ();
    return 0;
}
