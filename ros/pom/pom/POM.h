#pragma once

#include <opencv2/highgui/highgui.hpp>


#include "Pipeline2D.h"
#include "Object.h"
#include "commons.h"

//./bin/modelingPlanar /home/gmanfred/devel/datasets/my_objects/purfruit/
//                     /home/gmanfred/devel/datasets/my_objects/purfruit/corners.txt7
//                     /home/gmanfred/devel/datasets/my_objects/purfruit/dimensions.txt
class POM {
  public:
    POM ();
    void setIntrinsic (cv::Mat K);

    Object model (std::vector<cv::Mat> images, std::vector<std::vector<cv::Point2f> > corners,
                  cv::Point3f dimensions);
    void savePoints3d (std::vector<cv::Point3f> points3d, std::string filename);

  private:
    void extractFeatures (cv::Mat image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);
    void rectifyImage (int face, cv::Mat image, std::vector<cv::Point2f> corners2d, cv::Point3f dimensions,
                        cv::Mat &rectified_image);
    void convert2Dto3D (int face, cv::Mat image, cv::Point3f dimensions, std::vector<cv::Point2f> corners2d,
                         std::vector<cv::KeyPoint> keypoints,
                         std::vector<cv::Point3f> &points3d);
    void local2global (int face, cv::Point3f dimensions,
                        cv::Mat &R, cv::Mat &t);
    void convert2DtoLocal3D (cv::Mat image, cv::Point3f dimensions, std::vector<cv::KeyPoint> keypoints,
                              std::vector<cv::Point3f> &points3d);
    std::vector<cv::Point2f> getCorners (cv::Mat image);

    cv::Mat K_;
    // 2d to 3d transformation : f(x,y) = z
    Pipeline2D pipeline2d_;
};
