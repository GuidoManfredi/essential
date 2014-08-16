#pragma once

#include <iostream>
#include <cassert>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

class Pipeline2D {
  public:
    Pipeline2D ();

    void getGray(const cv::Mat& image, cv::Mat& gray);
    bool detectFeatures(const cv::Mat& image,
						std::vector<cv::KeyPoint>& keypoints);
    bool describeFeatures(const cv::Mat image, std::vector<cv::KeyPoint> keypoints,
						  cv::Mat& descriptors);
    bool extractFeatures(const cv::Mat& image,
  						 std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);
	bool match(const cv::Mat &desc1, const cv::Mat &desc2,
               std::vector<cv::DMatch>& matches);
    // Keeps only the matches_kept_ % closest matches
    void filterMatches (std::vector<cv::DMatch> matches,
                        std::vector<cv::DMatch> &filtered_matches);

  unsigned int minNumberMatchesAllowed_;

  private:
    struct lessThanDistance {
        inline bool operator() (const cv::DMatch &input1, const cv::DMatch &input2) {
            return (input1.distance < input2.distance);
        }
    };

    float matches_kept_; // percent of best matches kept by filtering

    cv::Ptr<cv::FeatureDetector>     detector_;
    cv::Ptr<cv::DescriptorExtractor> extractor_;
    cv::Ptr<cv::DescriptorMatcher>   matcher_;
};
