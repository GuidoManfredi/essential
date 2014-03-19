#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>

class IPRecognition
{
 public:
  IPRecognition ();
  ~IPRecognition ();
  
  void load_model (std::string filepath);
	void recognize(const cv::Mat img, const std::vector<std::string> candidates);
 private:
	void extract (const cv::Mat img,
			 					std::vector<cv::KeyPoint> &kpts,	cv::Mat &descs); 
  void match(const cv::Mat query, const cv::Mat train,
  					 std::vector<cv::DMatch>& matches);
	void local_spatial_verification ();
  void add_descriptors (cv::Mat in_descs);
  void symmetryCheck (std::vector< std::vector<cv::DMatch> > matches12,
                      std::vector< std::vector<cv::DMatch> > matches21,
                      std::vector< std::vector<cv::DMatch> > &filteredMatches);
  void ratioCheck (std::vector<std::vector<cv::DMatch> > &nmatches);
  int name2idx (std::string name);

	cv::Ptr<cv::FeatureDetector> _detector;
	cv::Ptr<cv::DescriptorExtractor> _extractor;
	cv::Ptr<cv::DescriptorMatcher> _matcher;

	std::vector<std::string> _names;
  std::vector<cv::Mat> _train_descs;
	std::vector<std::vector<cv::DMatch> > _knnMatches;
};
