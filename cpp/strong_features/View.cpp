#include "View.h"

using namespace std;
using namespace cv;

View::View() {}

View::View (const View &view) {
    image_ = view.image_;
    keypoints_ = view.keypoints_;
    descriptors_ = view.descriptors_;

    keys_ = view.keys_;
    tilt_ = view.tilt_;
    angle_ = view.angle_;
    points_ = view.points_;
}
////////////////////////////////////////////////////////////////////////////////
//  STORAGE FUNCTIONS
////////////////////////////////////////////////////////////////////////////////
void View::write(cv::FileStorage& fs) const {
    //cout << "Writing view" << endl;
    fs << "{" << "keypoints" << keypoints_
              << "descriptors" << descriptors_
              << "}";
}

void View::read(const FileNode& node) {
    //cout << "Reading view" << endl;
    FileNode keypoints = node["keypoints"];
    cv::read(keypoints, keypoints_);
    node["descriptors"] >> descriptors_;
}
