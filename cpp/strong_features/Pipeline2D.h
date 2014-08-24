#pragma once

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Containers.h"

class Pipeline2D {
  public:
    Pipeline2D ();

    void getGray(const cv::Mat& image, cv::Mat& gray);
    void extractDescriptors(const cv::Mat& image, const cv::Mat& mask, cv::Mat &descriptors);
	int match(const cv::Mat &desc1, const cv::Mat &desc2);

  private:
    void key2desc (std::vector<std::vector<keypointslist > > key, cv::Mat desc);

    cv::Ptr<cv::FeatureDetector>     detector_;
    cv::Ptr<cv::DescriptorExtractor> extractor_;
    cv::Ptr<cv::DescriptorMatcher>   matcher_;
};
