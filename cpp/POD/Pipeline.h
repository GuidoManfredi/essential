#ifndef POD_PIPELINE_H_
#define POD_PIPELINE_H_

//#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/nonfree/gpu.hpp>

#include "PlanarObject.h"

class Pipeline {
 public:
  Pipeline ();

  void getGray(const cv::Mat& image, cv::Mat& gray);
  bool gpu_extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const;
  bool extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const;
  void add_descriptors ();
  void getMatches(const cv::Mat& queryDescriptors, const cv::Mat& objectDescriptors,
                  std::vector<cv::DMatch>& matches);
  void getMatches(const cv::Mat& queryDescriptors, std::vector<cv::DMatch>& matches);
  bool refineMatchesWithHomography(
      const std::vector<cv::KeyPoint>& queryKeypoints,
      const std::vector<cv::KeyPoint>& trainKeypoints,
      float reprojectionThreshold,
      std::vector<cv::DMatch>& matches,
      cv::Mat& homography);
  bool find (const cv::Mat& image, PlanarObject& object);

  bool enableRatioTest;
  bool enableHomographyRefinement;
  float homographyReprojectionThreshold;
  int minNumberMatchesAllowed;

 private:
  std::vector<cv::KeyPoint> m_queryKeypoints;
  cv::Mat                   m_queryDescriptors;
  std::vector<cv::DMatch>   m_matches;
  std::vector< std::vector<cv::DMatch> > m_knnMatches;

  cv::Mat                   m_grayImg;
  cv::Mat                   m_warpedImg;
  cv::Mat                   m_roughHomography;
  cv::Mat                   m_refinedHomography;

  cv::Ptr<cv::FeatureDetector>     m_detector;
  cv::Ptr<cv::DescriptorExtractor> m_extractor;
  cv::Ptr<cv::DescriptorMatcher>   m_matcher;

  cv::gpu::SURF_GPU m_gpu_surf;
  //cv::gpu::BruteForceMatcher_GPU< cv::L2<float> > m_gpu_matcher;
};

#endif // POD_PIPELINE_H_

