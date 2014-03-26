#pragma once

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Pipeline2D.h"
#include "Pipeline2DGPU.h"

class Frame
{
 public:
	Frame ();
	void setMatchesLeft (std::vector<cv::DMatch> matches);
	void setMatchesRight (std::vector<cv::DMatch> matches);

	void filterNaNKeyPoints (cv::Mat depth,
	                         std::vector<cv::KeyPoint> kpts,
						     std::vector<cv::KeyPoint> &filtered_kpts,
						     std::vector<cv::Point3f> &filtered_p3d);

	void filterNaNKeyPoints (cv::Mat depth,
	                         std::vector<cv::KeyPoint> kpts,
						     std::vector<int> &filtered_kpts_index,
						     std::vector<cv::Point3f> &filtered_p3d);

    unsigned int id_;
    cv::Mat image_;
	cv::Mat gray_;
	std::vector<cv::KeyPoint> raw_kpts_;
	std::vector<cv::KeyPoint> kpts_; // removed those corresponding to NaN p3d
	cv::Mat descs_;
	
	std::vector<cv::Point3f> p3d_;

	std::vector<cv::DMatch> matches_left_;
	std::vector<cv::Point2f> p2d_left_;	

	std::vector<cv::DMatch> matches_right_;
	std::vector<cv::Point3f> p3d_right_;	
	
	cv::Mat pose_;
    
    int number_matches_first_match_;

 private:
};