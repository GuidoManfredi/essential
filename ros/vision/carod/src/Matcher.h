#ifndef DEF_MATCHER
#define DEF_MATCHER

#include <cstdio>

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define TWEEK_DETECTOR 400
#define TWEEK_DESCRIPTOR 32
#define TWEEK_MATCHER 10

class Matcher
{
public:
	Matcher();
	~Matcher();

	void extract (const cv::Mat img,
			 					std::vector<cv::KeyPoint> &kpts,	cv::Mat &descs);
  void set_descriptors (std::vector<cv::Mat> in_descs);
  void add_descriptors (std::vector<cv::Mat> in_descs);
  void add_kpts_descriptors (std::vector< std::vector<cv::KeyPoint> > in_kpts, std::vector<cv::Mat> in_descs);
	void match (cv::Mat descs1,	cv::Mat descs2,
							std::vector<int> &pts_idx1, std::vector<int> &pts_idx2);
  void match(const cv::Mat query, std::vector<cv::DMatch>& matches);

	cv::Mat get_image_matches (	const cv::Mat& img1, const cv::Mat& img2,
															std::vector<cv::KeyPoint> kpts1, std::vector<int> idx1,
															std::vector<cv::KeyPoint> kpts2, std::vector<int> idx2);

	void draw_matches ( const cv::Mat& img1, const cv::Mat& img2,
											std::vector<cv::KeyPoint> kpts1,
											std::vector<cv::KeyPoint> kpts2,
											std::vector<cv::DMatch> matches);
	void draw_matches ( const cv::Mat& img1, const cv::Mat& img2,
											std::vector<cv::KeyPoint> kpts1,
											std::vector<cv::DMatch> matches);
  void draw_kpts (const cv::Mat& img, std::vector<cv::KeyPoint> kpts);
private:
  void symmetryCheck (std::vector<cv::DMatch> matches12, std::vector<cv::DMatch> matches21,
                    std::vector<cv::DMatch> &filteredMatches);
  void symmetryCheck (std::vector< std::vector<cv::DMatch> > matches12,
                      std::vector< std::vector<cv::DMatch> > matches21,
                      std::vector< std::vector<cv::DMatch> > &filteredMatches);
  void ratioCheck (std::vector<std::vector<cv::DMatch> > &nmatches);

	cv::Ptr<cv::FeatureDetector> _detector;
	cv::Ptr<cv::DescriptorExtractor> _extractor;
	cv::Ptr<cv::DescriptorMatcher> _matcher;

  std::vector<cv::KeyPoint> _train_kpts;
  cv::Mat _train_descs;
	std::vector<std::vector<cv::DMatch> > _knnMatches;
};

#endif
