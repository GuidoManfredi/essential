#pragma once

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "Object.h"

enum Feature {eASIFT, eSIFT, eSURF, eORB, eFREAK, eBRISK}; // enum sift = eSIFT

typedef struct cModel {
    cv::Mat descriptors_;
    vector<vector<keypointslist> > keys_;
} Model;
/*
typedef struct cView {
    cView () {}

    cView (const cView &view) {
        tilt_ = view.tilt_;
        image_ = view.image_;
        angle_ = view.angle_;
        keypoints_ = view.keypoints_;
        descriptors_ = view.descriptors_;
        keys_ = view.keys_;
        points_ = view.points_;
    }

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
*/
// N = number of matches, P = percent of matches (N/nb kpts * 100)
// time is the time need to match a view to the model
typedef struct cError {
    int N_;
    float P_;
    float Rerr_;
    float time_;
} Error;


