#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

class PipelineGeom
{
  public:
    PipelineGeom ();
    void filterMatchesHomography (std::vector<cv::KeyPoint> keypoints1, std::vector<cv::KeyPoint> keypoints2,
                                  std::vector<cv::DMatch> &matches, cv::Mat &H);

  private:
    void match2points (std::vector<cv::KeyPoint> kpts1, std::vector<cv::KeyPoint> kpts2, std::vector<cv::DMatch> matches,
                       std::vector<cv::Point2f> &pts1, std::vector<cv::Point2f> &pts2);
};
