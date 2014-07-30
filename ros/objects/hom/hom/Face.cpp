#include "Face.h"

#include <iostream>

using namespace std;
using namespace cv;

Face::Face () {

}

Face::Face(cv::Mat img, int id, std::vector<cv::KeyPoint> kpts, cv::Mat descs):
image_(img), id_(id), keypoints_(kpts), tmp_keypoints_(kpts), descriptors_(descs), tmp_descriptors_(descs) {
    size_ = img.size();
    points3d_.clear();
    for (size_t i = 0; i < kpts.size(); ++i) {
        Point3f p3d;
        p3d.x = kpts[i].pt.x;
        p3d.y = kpts[i].pt.y;
        p3d.z = 0;
        points3d_.push_back(p3d);
    }
}

void Face::removeMatches(std::vector<cv::DMatch> matches) {
    // we assume trainIdx are our indexes (this face was 2nd in the match function)
    cout << "Removing keypoints from " << tmp_keypoints_.size();
    for (size_t i = 0; i < matches.size(); ++i) {
        int idx = matches[i].queryIdx; // query is the scene
        tmp_keypoints_.erase(tmp_keypoints_.begin() + idx);
        tmp_descriptors_.row(idx) = Mat::zeros(1, 128, CV_32F);
    }
    cout << " to " << tmp_keypoints_.size() << endl;
}

void Face::write(cv::FileStorage& fs) const {
    fs << "{" << "size" << size_
              << "points3d" << points3d_
              << "keypoints" << keypoints_
              << "descriptors" << descriptors_
              << "}";
}

void Face::read(const FileNode& node) {
    std::vector<float> size;
    node["size"] >> size;
    size_ = Size(size[0], size[1]);
    FileNode points3d = node["points3d"];
    cv::read(points3d, points3d_);
    FileNode keypoints = node["keypoints"];
    cv::read(keypoints, keypoints_);
    cv::read(keypoints, tmp_keypoints_);
    node["descriptors"] >> descriptors_;
}
