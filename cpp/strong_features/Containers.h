#pragma once

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "ASIFT/demo_lib_sift.h"

enum Feature {eASIFT, eSIFT, eSURF, eFREAK, eBRIEF}; // enum sift = eSIFT

typedef struct cModel {
    cv::Mat descriptors_;
    vector<vector<keypointslist> > keys_;
} Model;

typedef struct cView {
    int tilt_;
    cv::Mat image_;
    float angle_;
    vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;
    vector<vector<keypointslist> > keys_;
    vector<cv::Point3f> points_;
} View;

typedef struct cObject {
    std::vector<View> views_;
} Object;

// N = number of matches, P = percent of matches (N/nb kpts * 100)
typedef struct cError {
    int N_;
    float P_;
    float Rerr_;
} Error;


