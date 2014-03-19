#ifndef _POM3_FACE_H_
#define _POM3_FACE_H_

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

class Face
{
 public:
  Face();

  std::vector<cv::Mat> views_;
  std::vector<std::vector<cv::KeyPoint> > keypoints_;
  std::vector<cv::Mat> descriptors_;
  std::vector<cv::Mat> bows_;
  cv::Mat vocabulary_;

 private:
};

#endif // _POM3_FACE_H_
