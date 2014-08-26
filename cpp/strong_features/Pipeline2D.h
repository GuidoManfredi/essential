#pragma once

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Containers.h"

#include "ASIFT/compute_asift_keypoints.h"
#include "ASIFT/compute_asift_matches.h"

class Pipeline2D {
  public:
    Pipeline2D ();

    void getGray(const cv::Mat& image, cv::Mat& gray);

    void extractDescriptors(const cv::Mat& image, const cv::Mat& mask,
                             std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);
	int match(const cv::Mat &desc1, const cv::Mat &desc2);

    void extractDescriptors(const cv::Mat& image, const cv::Mat& mask,
                             vector<vector<keypointslist > > &keys);
    int match (std::vector<std::vector<keypointslist > > keys1,
                std::vector<std::vector<keypointslist > > keys2,
                int width1, int height1, int width2, int height2);

  private:
    cv::Ptr<cv::FeatureDetector>     detector_;
    cv::Ptr<cv::DescriptorExtractor> extractor_;
    cv::Ptr<cv::DescriptorMatcher>   matcher_;
};
