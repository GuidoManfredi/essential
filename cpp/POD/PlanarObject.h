#ifndef POD_PLANAR_OBJECT_H_
#define POD_PLANAR_OBJECT_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

class PlanarObject {
 public:
  PlanarObject ();

  cv::Size                  size;
  cv::Mat                   frame;
  cv::Mat                   gray;

  std::vector<cv::KeyPoint> keypoints;
  cv::Mat                   descriptors;

  std::vector<cv::Point2f>  corners2d;
  std::vector<cv::Point3f>  corners3d;

  cv::Mat                   homography;
  std::vector<cv::Point2f>  points2d;
  cv::Mat                   pose3d;
};


#endif // POD_PLANAR_OBJECT_H_
