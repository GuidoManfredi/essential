#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

class Face {
  public:
    Face();
    Face(cv::Mat img, std::vector<cv::KeyPoint> kpts, cv::Mat descs);
    
  private:
    cv::Mat image_;
    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;
};
