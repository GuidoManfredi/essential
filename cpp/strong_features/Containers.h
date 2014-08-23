#pragma once

#include <vector>
#include <opencv2/core/core.hpp>

typedef struct cModel {
    cv::Mat descriptors_;
} Model;

typedef struct cView {
    float angle_;
    cv::Mat descriptors_;
} View;

typedef struct cObject {
    std::vector<View> views_;
} Object;
