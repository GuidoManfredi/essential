#ifndef DEF_CAROD_DETECTOR
#define DEF_CAROD_DETECTOR

#include "opencv2/calib3d/calib3d.hpp"

#include "Object.h"
#include "Matcher.h"
#include "PoseEstimator.h"

class Detector
{
public:
  void train_descs (std::vector<cv::Mat> descs);
  void train_kpts_descs (std::vector< std::vector<cv::KeyPoint> > kpts,
                          std::vector<cv::Mat> descs);
  cv::Matx34d compute_pose (cv::Mat img, std::vector<cv::Point3d> p3d);
  void openMVG_function ();
  Detector (cv::Mat K, cv::Mat d);
  ~Detector ();

private:
	Matcher _matcher;
	PoseEstimator* pose_estimator_;

	cv::Matx34d P_;
};

#endif
