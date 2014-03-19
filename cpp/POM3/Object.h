#pragma once

#include "Face.h"

class Object
{
  public:
    Object ();
    void addViews (std::vector<cv::Mat> views);
    void addFace (Face face);

    std::vector<cv::Mat> views_;
    std::vector<std::vector<cv::KeyPoint> > keypoints_;
    std::vector<cv::Mat> descriptors_;
    std::vector<cv::Mat> bows_;
    cv::Mat vocabulary_;
};
