#ifndef HAND_DETECT_DETECTOR_H_
#define HAND_DETECT_DETECTOR_H_

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

class Detector
{
 public:
  Detector (std::string cascade_name);
	std::vector<cv::Rect> detect (cv::Mat img);
	cv::Rect detect (cv::Mat img, cv::Rect loc);
 
 private:
	cv::CascadeClassifier cascade_;
	std::string cascade_name_;
};

#endif
