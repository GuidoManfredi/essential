// For informations about opencv and histograms, refer to this page
// http://docs.opencv.org/doc/tutorials/imgproc/histograms/histogram_comparison/histogram_comparison.html
#include <opencv2/imgproc/imgproc.hpp>

class RecoHist
{
 public:
 	RecoHist ();
 	// Assume image is rgb
 	std::vector<std::string> recognize (cv::Mat img, double thresh,
 																			std::vector<std::string> candidates = std::vector<std::string>());
 	void load_training_file (std::string filepath);
 private:
	std::vector<std::string> recognize_hist (cv::Mat hist, double thresh,
 														  						 std::vector<std::string> candidates = std::vector<std::string>());
 	double distance (cv::Mat hist1, cv::Mat hist2);

	// Histograms of known objects
 	std::vector<cv::Mat> _vec_hist;
 	// Names of known objects
 	std::vector<std::string> _vec_names;
 	// 0=correlation,  1=chi-squared,  2=intersection,  3=bhattacharyya
 	int _compare_method;
};

// TODO implementer load_training_file.
