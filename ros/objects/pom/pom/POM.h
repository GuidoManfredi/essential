#pragma once

#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "Pipeline2D.h"
#include "Object.h"
#include "commons.h"

// For PAVE dimensions are width, height and depth.
// for CYL, it is Radius and height, last coordinate is not used and set to zero.
enum SHAPE {PAVE, CYL};

class POM {
  public:
    POM ();
    /**
    *   \Brief From a set of images, known corners and dimensions of object and
    *           known shape, this function creates a model ready to be used for
    *           recognition and localisation. It also outputs a .pcd file to
    *           visualize the objects 3D points.
    */
    Object model (SHAPE shape, std::vector<cv::Mat> images, std::vector<int> faces, std::vector<std::vector<cv::Point2f> > corners,
                  cv::Point3f dimensions);
    // for debug only
    void savePoints3d (std::vector<cv::Point3f> points3d, std::vector<cv::Vec3b> color, std::string filename);

  private:
    void saveColor(cv::Mat image, std::vector<cv::KeyPoint> kpts, std::vector<cv::Vec3b> &color);
    void extractFeatures (cv::Mat image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);
    // Faces, 0:front, 1:left, 2:right, 3:back, 4:top, 5:bottom (ordre pour des questions historique : le premier objet avait un back et front identique, mais
    // on ne peut pas mettre deux fois la même image car SiftGpu n'aime pas ça, il n'extrait pas de descripteurs).
    void rectifyImage (cv::Mat image, std::vector<cv::Point2f> corners2d, cv::Point3f dimensions,
                        cv::Mat &rectified_image);
    // 2d to 3d transformation : f(x,y) = z
    void convert2Dto3D (SHAPE shape, int face, cv::Mat image, cv::Point3f dimensions, std::vector<cv::Point2f> corners2d,
                         std::vector<cv::KeyPoint> keypoints,
                         std::vector<cv::Point3f> &points3d);
    void local2global (SHAPE shape, int face, cv::Point3f dimensions,
                        cv::Mat &R, cv::Mat &t);
    void local2globalPave (int face, cv::Point3f dimensions,
                            cv::Mat &R, cv::Mat &t);
    void local2globalCyl (int face, cv::Point3f dimensions,
                            cv::Mat &R, cv::Mat &t);
    void convert2DtoLocal3D (SHAPE shape, int face, cv::Mat image, cv::Point3f dimensions, std::vector<cv::KeyPoint> keypoints,
                              std::vector<cv::Point3f> &points3d);
    void convert2Dto3Dpave (int face, cv::Mat image, cv::Point3f dimensions, std::vector<cv::KeyPoint> keypoints,
                            std::vector<cv::Point3f> &points3d);
    void convert2Dto3Dcyl (int face, cv::Mat image, cv::Point3f dimensions, std::vector<cv::KeyPoint> keypoints,
                           std::vector<cv::Point3f> &points3d);
    std::vector<cv::Point2f> getCorners (std::vector<cv::Point2f> corners2d);

    Pipeline2D pipeline2d_;
};
