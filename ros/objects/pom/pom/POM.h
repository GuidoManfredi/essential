#pragma once

#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "Pipeline2D.h"
#include "Object.h"
#include "commons.h"

class POM {
  public:
    POM ();
    // In the dimensions.txt order is : width, height and depth.
    Object model (std::vector<cv::Mat> images, std::vector<int> faces, std::vector<std::vector<cv::Point2f> > corners,
                  cv::Point3f dimensions);
    // for debug only
    void savePoints3d (std::vector<cv::Point3f> points3d, std::string filename);

  private:
    void extractFeatures (cv::Mat image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);
    // Faces, 0:front, 1:left, 2: right, 3:back, 4:top, 5:bottom (ordre pour des questions historique : le premier objet avait un back et front identique, mais
    // on ne peut pas mettre deux fois la même image car SiftGpu n'aime pas ça, il n'extrait pas de descripteurs).
    void rectifyImage (cv::Mat image, std::vector<cv::Point2f> corners2d, cv::Point3f dimensions,
                        cv::Mat &rectified_image);
    // 2d to 3d transformation : f(x,y) = z
    void convert2Dto3D (int face, cv::Mat image, cv::Point3f dimensions, std::vector<cv::Point2f> corners2d,
                         std::vector<cv::KeyPoint> keypoints,
                         std::vector<cv::Point3f> &points3d);
    void local2global (int face, cv::Point3f dimensions,
                        cv::Mat &R, cv::Mat &t);
    void convert2DtoLocal3D (int face, cv::Mat image, cv::Point3f dimensions, std::vector<cv::KeyPoint> keypoints,
                              std::vector<cv::Point3f> &points3d);
    std::vector<cv::Point2f> getCorners (std::vector<cv::Point2f> corners2d);

    Pipeline2D pipeline2d_;
};
