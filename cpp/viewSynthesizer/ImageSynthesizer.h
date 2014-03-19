#ifndef POD_IMAGE_SYNTHESIZER_H_
#define POD_IMAGE_SYNTHESIZER_H_

#include <cstdio>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class ImageSynthesizer{
 public:
 void views (cv::Mat img, std::vector<cv::Point2f> in_corners,
              std::vector<cv::Mat> &views,
              std::vector< std::vector<cv::Point2f> > &corners);
  void generateFrontalView (cv::Mat in, std::vector<cv::Point2f> corners,
                            cv::Mat &out,
                            std::vector<cv::Point2f> &out_corners);
  // Each angle and scale has three values, e.g. for yaw :
  // yaw.x = starting angle
  // yaw.y = step
  // yaw.z = final angle
  // For scale it is : starting scale/scale factor/ending scale
  void generateViews (const cv::Mat img,
                        cv::Point3f yaw, cv::Point3f pitch, cv::Point3f roll, cv::Point3f scale,
                        std::vector<cv::Mat> &warped_img,
                        std::vector< std::vector<cv::Point2f> > &warped_corners);
  void generateViewsCollet (const cv::Mat img,
                            std::vector<cv::Point2f> corners,
                            cv::Point3f theta,
                            std::vector<cv::Mat> &warped_img,
                            std::vector< std::vector<cv::Point2f> > &warped_corners);
 //private:
  void warpImage(const cv::Mat &in, cv::Mat &out, float yaw, float pitch, float roll,
                  cv::Mat &affine);
  void warpImageProjective(const cv::Mat &in, cv::Mat &out, float yaw, float pitch, float roll);
  void warpImageCollet(const cv::Mat &in, cv::Mat &out, const std::vector<cv::Point2f> &corners, std::vector<cv::Point2f> &warped_corners,
                       float yaw, float pitch, float roll,
                       cv::Mat &affine);
  // makes a 3x3 rotation matrix, inputs are in radians
  cv::Mat makeRotation(float yaw, float pitch, float roll);
  cv::Mat makeAffine(float phi, float theta, float psi, float lambda);
};
#endif // POD_IMAGE_SYNTHESIZER_H_

