#include "hgd.h"

using namespace std;
using namespace cv;

void drawPath (cv::Mat img, cv::Rect pose) {
    Point center = Point (pose.width - pose.x, pose.height - pose.y);
    cout << "center :" << center << endl;
    circle(img, center, radius, Scalar(255, 255, 255), -1);
}
