#pragma once

#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>

#include "Object.h"

class PipelineGeom
{
  public:
    PipelineGeom ();
    void mySolvePnP (cv::Mat points3d, cv::Mat points2d, cv::Mat K,
                     cv::Mat &Rov, cv::Mat &tov,
                     cv::Mat &p3d_inliers, cv::Mat &p2d_inliers);
    void pointsFromIndex (cv::Mat p3d, cv::Mat p2d, std::vector<int> inliers,
                           cv::Mat &p3d_inliers, cv::Mat &p2d_inliers);
};
