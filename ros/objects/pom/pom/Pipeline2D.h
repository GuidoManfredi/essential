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
    bool detectFeaturesGpu(const cv::Mat& image,
						std::vector<cv::KeyPoint>& keypoints);
    bool describeFeaturesGpu(const cv::Mat image, std::vector<cv::KeyPoint> keypoints,
						  cv::Mat& descriptors);
    bool detectFeatures(const cv::Mat& image,
						std::vector<cv::KeyPoint>& keypoints);
    bool describeFeatures(const cv::Mat image, std::vector<cv::KeyPoint> keypoints,
						  cv::Mat& descriptors);
    bool extractFeatures(const cv::Mat& image, bool gpu,
  						 std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);
	bool match(const cv::Mat &desc1, const cv::Mat &desc2, bool gpu,
               std::vector<cv::DMatch>& matches);
	void matchGpu (const cv::Mat &desc1, const cv::Mat &desc2,
                         std::vector<cv::DMatch>& matches);
	void matchCV (const cv::Mat &desc1, const cv::Mat &desc2,
                         std::vector<cv::DMatch>& matches);
    int getWordFromDescriptorIndex (std::vector<std::vector<int> > query_indices, int i);
    cv::Mat getDescriptorsFromIndices (cv::Mat train_descriptors,
                                       std::vector<int> train_descriptors_indices);

    unsigned int minNumberMatchesAllowed_;

  private:
    struct IndexDistanceComparatorClass {
        bool operator() (const IndexDistance& l, const IndexDistance& r) {
            //std::cout << l.second << " " << r.second << std::endl;
            return l.second < r.second;
        }
    } IndexDistanceComparator;

    cv::KeyPoint GPUkpt2kpt (SiftGPU::SiftKeypoint key);
    cv::Mat GPUdesc2desc (std::vector<float> descriptors);
    cv::Mat GPUdesc2desc (std::vector<float> descriptors, std::vector<int> kpts_index);
    std::vector<float> desc2GPUdesc (cv::Mat descs);

    SiftGPU sift_;
    SiftMatchGPU matcher_gpu_;
    
    cv::Ptr<cv::FeatureDetector>     detector_;
    cv::Ptr<cv::DescriptorExtractor> extractor_;
    cv::Ptr<cv::DescriptorMatcher>   matcher_;
};
