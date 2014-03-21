#pragma once

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

class PipelineGeom
{
  public:
    PipelineGeom (std::string calibration_file);
    cv::Mat filterMatchesHomography (std::vector<cv::KeyPoint> keypoints1, std::vector<cv::KeyPoint> keypoints2,
                                  std::vector<cv::DMatch> &matches);
    cv::Mat filterMatchesPnP (std::vector<cv::Point3f> point3d, std::vector<cv::KeyPoint> keypoints,
                              std::vector<cv::DMatch> &matches);
    cv::Mat computePnP (std::vector<cv::Point2f> corners2d, cv::Mat H);
    cv::Mat warpImage (cv::Mat image, cv::Mat H, cv::Size size);
  private:
    void match2points (std::vector<cv::KeyPoint> kpts1, std::vector<cv::KeyPoint> kpts2, std::vector<cv::DMatch> matches,
                       std::vector<cv::Point2f> &pts1, std::vector<cv::Point2f> &pts2);
    void match2points (std::vector<cv::Point3f> kpts1, std::vector<cv::KeyPoint> kpts2, std::vector<cv::DMatch> matches,
                       std::vector<cv::Point3f> &pts1, std::vector<cv::Point2f> &pts2);
    cv::Mat vecToMat (cv::Mat rvec, cv::Mat tvec);

    cv::Mat K_, d_;
};
