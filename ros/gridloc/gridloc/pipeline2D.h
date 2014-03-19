#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>


class Pipeline2D
{
  public:
    Pipeline2D ();
    void color2gray (cv::Mat image, cv::Mat &gray);

    int detectCircles (cv::Mat image, std::vector<cv::KeyPoint> &circles);
    void describeCircles (cv::Mat image, std::vector<cv::KeyPoint> circles, cv::Mat &descriptors);
    int extractCircles (cv::Mat image, std::vector<cv::KeyPoint> &circles, cv::Mat &descriptors);

    void match (cv::Mat desriptors_query, cv::Mat descriptors_train, std::vector<cv::DMatch> &matches);

    void drawCircles (cv::Mat &image, std::vector<cv::KeyPoint> circles);
  private:
    int findCircles (cv::Mat image, std::vector<cv::KeyPoint> &keypoints);
    int removeNonRemarquableCircles (std::vector<cv::KeyPoint> circles,
                                     std::vector<cv::KeyPoint> &remarquable_circles);
};
