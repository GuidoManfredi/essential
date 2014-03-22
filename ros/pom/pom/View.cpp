#include "View.h"

using namespace std;
using namespace cv;

View::View() {}

Mat View::transform () {
    return Rt2P (R_, t_);
}

void View::write(cv::FileStorage& fs) const {
    fs << "{" << "number_points" << number_points_;
    for ( int i = 0; i < number_points_; ++i ) {
        fs << "points" << points3d_;
        fs << "descriptors" << descriptors_;
    }
    fs << "}";
}

void View::read(const FileNode& node) {
    node["number_points"] >> number_points_;
    FileNode points3d = node["points3d"];
    cv::read(points3d, points3d_);
    node["descriptors"] >> descriptors_;
}

