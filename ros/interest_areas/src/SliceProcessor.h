#pragma once
#include <iostream>
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



class SliceProcessor
{
	public:
		SliceProcessor ();
		std::vector< std::vector<cv::Point> > compute_interest_area_contours (std::vector<cv::Mat> slices,
																																					unsigned int thresh);
		cv::Mat denoise (cv::Mat slice);
		std::vector< std::vector<cv::Point> > compute_contours (cv::Mat slice);
		cv::Mat contours_to_mat (std::vector< std::vector<cv::Point> > contours);
		cv::Mat contours_to_mat (std::vector<cv::Point> contours);
		//void save_slices (std::vector< std::vector<cv::Point> > contours, std::string path);
		void save_slice (cv::Mat contours, unsigned int i);

		std::vector<cv::Vec4i> segment_lines (cv::Mat img,
																					std::vector<cv::Point> &all_inliers);
		cv::Vec4i segment_line_ransac (cv::Mat img,
																	unsigned int inlier_tresh,
																	unsigned int num_iterations,
																	std::vector<cv::Point> &inliers);
		cv::Vec4i get_two_points_random (unsigned int width, unsigned int height);
		//cv::Vec4i get_two_points_random ();
		std::vector<cv::Point> get_inliers (cv::Mat img, 
																				cv::Vec4i line, unsigned int tresh);
		void remove (cv::Mat &img, std::vector<cv::Point> pts);
		cv::Vec4i min_max_along_line (std::vector<cv::Point> vec, cv::Vec4i line);
		void min_max_along_line (std::vector<cv::Point> vec, cv::Vec4i line,
														 cv::Point &min, cv::Point &max);
		std::vector<cv::Point> get_occupied_points (cv::Mat img);
		std::vector<cv::Vec4i> merge_close_segments (std::vector<cv::Vec4i> segments);
		void add_to_group (cv::Vec4i in, cv::Vec4i &group, unsigned int &size);
		double segment_distance (cv::Vec4i seg1, cv::Vec4i seg2);
		
		void get_bounding_lines (cv::Vec4i segment, double delta,
														int width, int height,
														 cv::Vec4i &line1, cv::Vec4i &line2);
		bool is_up (cv::Vec4i line, cv::Point p);
		cv::Vec4i normal_at_point (double x, double y, double delta,
																 double a, double b);
		void normal_equation (double x, double y, double a1, double b1,
														double &a2, double &b2);
		void segment_equation (cv::Vec4i segment, double &a, double &b);
	private:
		unsigned int width_, height_;
		std::vector<cv::Point> occupied_points_;
};
