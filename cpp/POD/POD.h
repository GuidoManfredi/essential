#ifndef POD_POD_H_
#define POD_POD_H_

#include "PlanarObject.h"
#include "Pipeline.h"
// Planar Object Detector
class POD
{
 public:
  POD (cv::Mat K);
  //bool load_file (std::string image_path);
  //bool load_directory (std::string dir_path);
  void train(const PlanarObject& object);
  //bool findPattern(const cv::Mat& image, PatternTrackingInfo& info);

 private:
  cv::Mat K_;
  std::vector<PlanarObject> objects_;

  Pipeline pipeline_;

  std::vector<cv::KeyPoint> query_kpts_;
  cv::Mat                   query_descs_;
  std::vector<cv::DMatch>   matches_;
  std::vector< std::vector<cv::DMatch> > knnMatches_;
};

#endif // POD_POD_H_
