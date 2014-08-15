#pragma once

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class HandSeg {
  public:
    HandSeg();
    void getHands(std::vector<cv::Mat> images,
                   std::vector<cv::Mat> &hands);
    cv::Mat getHand(cv::Mat image);
    cv::Rect getHandBox (cv::Mat image);

  private:
    void segmentSkinHSV(cv::Mat image, cv::Mat min_hsv, cv::Mat max_hsv,
                        cv::Mat &mask);
    void segmentSkinGM(cv::Mat image, cv::Mat min_hsv, cv::Mat max_hsv,
                        cv::Mat &mask);
};
