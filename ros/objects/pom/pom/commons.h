#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

cv::Mat rt2P (cv::Mat rvec, cv::Mat tvec);
cv::Mat Rt2P (cv::Mat R, cv::Mat tvec);
