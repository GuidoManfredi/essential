#pragma once

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "ASIFT/demo_lib_sift.h"

typedef struct cModel {
    cv::Mat descriptors_;
    vector<vector<keypointslist> > keys_;
} Model;

typedef struct cView {
    float angle_;
    vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;
    vector<vector<keypointslist> > keys_;
    vector<cv::Point3f> points_;
    int width_;
    int height_;
} View;

typedef struct cObject {
    std::vector<View> views_;
} Object;
