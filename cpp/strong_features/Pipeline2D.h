#pragma once

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "Containers.h"

#include "ASIFT/compute_asift_keypoints.h"
#include "ASIFT/compute_asift_matches.h"

class Pipeline2D {
  public:
    Pipeline2D (cv::Mat K);
    void setFeatures (Feature ft);
    int getFeatures ();
    void getGray(const cv::Mat& image, cv::Mat& gray);

    void extractDescriptors(const cv::Mat& image, const cv::Mat& mask,
                             std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);

	int match(const cv::Mat &desc1, const cv::Mat &desc2, std::vector<cv::DMatch> &matches);
    float estimate_pose (std::vector<cv::KeyPoint> keypoints, std::vector<cv::Point3f> points,
                          std::vector<cv::DMatch> &matches);
    float estimate_pose2 (std::vector<cv::KeyPoint> keypoints1, std::vector<cv::KeyPoint> keypoints2,
                          std::vector<cv::DMatch> &matches);
    float dist_angle (float a, float b);

  private:
    cv::Mat K_;
    cv::Ptr<cv::FeatureDetector>     detector_;
    cv::Ptr<cv::DescriptorExtractor> extractor_;
    cv::Ptr<cv::DescriptorMatcher>   matcher_;
    Feature features_type;
    bool ASIFT_;

    void key2desc (std::vector<std::vector<keypointslist > > key, cv::Mat &desc);
    void key2kpts (std::vector<std::vector<keypointslist > > key, std::vector<cv::KeyPoint> &kpts);
    void get_matched_point(std::vector<cv::KeyPoint> keypoints, std::vector<cv::Point3f> points, std::vector<cv::DMatch> matches,
                            std::vector<cv::Point2f> &kpts, std::vector<cv::Point3f> &pts);
    void get_matched_keypoint(std::vector<cv::KeyPoint> keypoints1, std::vector<cv::KeyPoint> keypoints2, std::vector<cv::DMatch> matches,
                                std::vector<cv::Point2f> &kpts1, std::vector<cv::Point2f> &kpts2);
};
