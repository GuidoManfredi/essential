#pragma once

#include <iostream>
#include <cassert>

#include <GL/glew.h>
#include <GL/glut.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "/home/gmanfred/devel/sandbox/SiftGPU/src/SiftGPU/SiftGPU.h"

class Pipeline2DGPU {
  public:
    Pipeline2DGPU ();

    void getGray(const cv::Mat& image, cv::Mat& gray);
    bool detectFeatures(const cv::Mat& image,
						std::vector<cv::KeyPoint>& keypoints);
	// Be carefull with this one, as keypoints are ignore and we suppose keypoints
	// are already in memory, this is potentialy dangerous.
    bool describeFeatures(const cv::Mat image, std::vector<cv::KeyPoint> keypoints,
						  cv::Mat& descriptors);
    bool describeFeatures(const cv::Mat image, std::vector<int> keypoints_index,
						  cv::Mat& descriptors);						  
	bool match(const cv::Mat &desc1, const cv::Mat &desc2,
               std::vector<cv::DMatch>& matches);
                   
  unsigned int minNumberMatchesAllowed_;

  private:
    cv::KeyPoint GPUkpt2kpt (SiftGPU::SiftKeypoint key);
    cv::Mat GPUdesc2desc (std::vector<float> descriptors);
    cv::Mat GPUdesc2desc (std::vector<float> descriptors, std::vector<int> kpts_index);
    std::vector<float> desc2GPUdesc (cv::Mat descs);
    
    struct lessThanDistance {
        inline bool operator() (const cv::DMatch &input1, const cv::DMatch &input2) {
            return (input1.distance < input2.distance);
        }
    };
  
    float matches_kept_; // percent of best matches kept by filtering
    
    //create a SiftGPU instance 
    SiftGPU sift;
    //SiftMatchGPU matcher;
};
