#pragma once

#include <vector>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define PNP_POINTS 4

class PipelineGeom {
  public:
    PipelineGeom (std::string calibration_file,
                  unsigned int required_inliers);
    cv::Mat computePose (std::vector<cv::Point3f> p3d,
                           std::vector<cv::Point2f> p2d,
                           std::vector<int> &inliers);
    double meanReprojectionError (std::vector<cv::Point3f> p3d,
                                  std::vector<cv::Point2f> p2d,
                                  cv::Mat R, cv::Mat t);
  private:
	unsigned int required_inliers_;
    cv::Mat K_;
    cv::Mat d_;
};
