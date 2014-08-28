#pragma once

#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>

class HaarDetect {
  public:
    HaarDetect();
    cv::Rect detect(cv::Mat frame, cv::Point2f clue, int padding, bool display);
    int loadCascade(std::string cascade_path);

  private:
    std::string face_cascade_name_;
    cv::CascadeClassifier face_cascade_;
};
