#pragma once

#include <vector>
#include <opencv2/core/core.hpp>

#include "ASIFT/demo_lib_sift.h"

typedef struct cModel {
    cv::Mat descriptors_;
    vector<vector<keypointslist> > keys_;
} Model;

typedef struct cView {
    float angle_;
    cv::Mat descriptors_;
    vector<vector<keypointslist> > keys_;
    int width_;
    int height_;
} View;

typedef struct cObject {
    std::vector<View> views_;
} Object;
