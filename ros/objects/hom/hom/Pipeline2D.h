#pragma once

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#define USE_GPU

#ifdef USE_GPU
#include <GL/glew.h>
#include <GL/glut.h>
#include "/home/gmanfred/devel/sandbox/SiftGPU/src/SiftGPU/SiftGPU.h"
#endif

typedef std::pair<int,double> IndexDistance;

class Pipeline2D {
  public:
    Pipeline2D ();

    void getGray(const cv::Mat& image, cv::Mat& gray);
    std::vector<cv::Point2f> getCorners(cv::Mat image);
    // Features functions
    bool detectFeatures(const cv::Mat& image,
						std::vector<cv::KeyPoint>& keypoints);
    bool describeFeatures(const cv::Mat image, std::vector<cv::KeyPoint> keypoints,
						  cv::Mat& descriptors);
    bool extractFeatures(const cv::Mat& image,
  						 std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);
	bool match(const cv::Mat &desc1, const cv::Mat &desc2,
               std::vector<cv::DMatch>& matches);

    bool ratio_test_;
    unsigned int minNumberMatchesAllowed_;

  private:
    cv::Ptr<cv::FeatureDetector>     detector_;
    cv::Ptr<cv::DescriptorExtractor> extractor_;
    cv::Ptr<cv::DescriptorMatcher>   matcher_;

    std::vector<cv::DMatch>   matches_;
    std::vector< std::vector<cv::DMatch> > knnMatches_;
};
