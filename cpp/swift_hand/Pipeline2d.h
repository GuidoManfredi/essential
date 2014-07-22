#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

class Pipeline2d {
    Pipeline2d();

    void getBoF (cv::Mat img,
                  cv::Mat &bof);

  private:
    cv::Mat vocabulary;
    cv::Ptr<cv::FeatureDetector> _detector;
	cv::Ptr<cv::DescriptorExtractor> _extractor;
	cv::Ptr<cv::DescriptorMatcher> _matcher;

	int _min_matches_allowed;
};
