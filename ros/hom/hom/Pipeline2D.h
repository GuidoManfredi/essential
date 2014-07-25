#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Pipeline2D {
  public:

  private:
    int extractFeatures(cv::Mat image, 
                    std::vector<cv::KeyPoint> &kpts, cv::Mat &descriptors);
    int match(cv::Mat descs1, cv::Mat descs2, 
              std::vector<cv::DMatch> &matches);
    int computeHomography();
};


