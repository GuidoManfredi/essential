#pragma once

#include <iostream>
#include <opencv2/core/core.hpp>

class HGD
{
  public:
    HGD ();
    void drawPath (cv::Mat img, cv::Rect pose);
};
