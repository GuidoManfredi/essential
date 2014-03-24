#include "View.h"

using namespace std;
using namespace cv;

View::View() {
    R_ = Mat::eye (3, 3, CV_32F);
    t_ = Mat::zeros (1, 3, CV_32F);
}

Mat View::transform () {
    return Rt2P (R_, t_);
}

void View::write(cv::FileStorage& fs) const {
    fs << "{" << "points3d" << points3d_
              << "descriptors" << descriptors_
              << "}";
}

void View::read(const FileNode& node) {
    FileNode points3d = node["points3d"];
    cv::read(points3d, points3d_);
    node["descriptors"] >> descriptors_;
}

