#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

class Histogram
{
 public:
 	Histogram () {}
	cv::Mat selective_spatial_histogram (cv::Mat img, int nbins);
	cv::Mat spatial_histogram (cv::Mat img, int nbins);
	cv::Mat classic_histogram (cv::Mat img, cv::Mat mask, int nbins);
 private:
	 cv::Mat compute_selective_descriptor (cv::Mat img, std::vector<cv::Mat> II);
	 cv::Mat compute_descriptor (std::vector<cv::Mat> II);
	 cv::Mat compute_classes_image (cv::Mat img, int nbins);
	 
 	 float* get_ranges(int nbins, int min, int max);
	 int get_bin (int val, float* ranges, int nbins);
	 int* compute_spatial_quadrants (cv::Mat II, int i, int j);
	 std::vector<cv::Mat> integral_images (cv::Mat classes, int nbins);
	 std::vector<cv::Mat> selective_cumsum_rows (cv::Mat img, int nbins);
	 std::vector<cv::Mat> selective_cumsum_cols (std::vector<cv::Mat> I, int nbins);
};
