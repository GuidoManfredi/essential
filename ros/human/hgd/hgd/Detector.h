#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class Detector
{
  public:
    Detector (std::string cascade_name);
	std::vector<cv::Rect> detect (cv::Mat img);
    cv::Rect detectOne (cv::Mat img);

  private:
	cv::CascadeClassifier cascade_;
	std::string cascade_name_;
};

