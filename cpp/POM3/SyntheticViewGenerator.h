#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>

class SyntheticViewGenerator {
  public:
    SyntheticViewGenerator();
    int generateViews (cv::Mat image, std::vector<cv::Point2f> corners,
                       std::vector<cv::Mat> &views, std::vector<cv::Mat> &affines);
    cv::Mat generateFrontalView (cv::Mat in, std::vector<cv::Point2f> corners);

    std::vector<cv::Point2f> getCorners (cv::Mat image);
    std::vector<cv::Point2f> getCorners (float width, float height);

  private:
  // Each angle and scale has three values, e.g. for yaw :
  // yaw.x = starting angle
  // yaw.y = step
  // yaw.z = final angle
    void generateAffineViews (cv::Mat image, cv::Point3f theta,
                              std::vector<cv::Mat> &views, std::vector<cv::Mat> &affines);
    cv::Mat generateAffineView (const cv::Mat image,
                                float theta, float phi, float psi,
                                cv::Mat &affine);
    cv::Mat warpImage(const cv::Mat image, const std::vector<cv::Point2f> corners,
                      float theta, float phi, float psi,
                      cv::Mat &affine3x2);
    cv::Mat makeAffine(float phi, float theta, float psi, float lambda);
    double b_;
};

