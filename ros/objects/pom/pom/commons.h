#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#ifndef USE_SIFTGPU
#define USE_SIFTGPU true
#endif

cv::Mat rt2P (cv::Mat rvec, cv::Mat tvec);
cv::Mat Rt2P (cv::Mat R, cv::Mat tvec);
