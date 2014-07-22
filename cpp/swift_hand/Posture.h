#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

class Posture {
  public:
    Posture();
    int save(std::string path);
    int load(std::string path);

  //private:
    cv::Mat descs_;
    std::vector<cv::KeyPoint> kpts_;
    int id;
};
