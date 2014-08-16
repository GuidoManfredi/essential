#pragma once

#include "tools.h"
#include "Pipeline2D.h"
#include <pcl/registration/icp.h>

// For each new frame
    // Acquiere pointcloud and transform
    // Get descriptors
    // Apply acquiered transform to pointcloud
    // Register new pointcloud to existing model with ICP
    // Add 3D points and descriptors to model
    // Remove close points

// Cheating Object Modeling
class COM {
  public:
    COM ();
    void addFrame (cv::Mat image, cv::Mat depth, cv::Mat P);
    std::vector<cv::Point3f> p3d();
    void saveModel(std::string filepath);

  private:
    void transform (std::vector<cv::Point3f> pts_in, cv::Mat P, std::vector<cv::Point3f> &pts_out);
    void refine (std::vector<cv::Point3f> pts1, std::vector<cv::Point3f> pts2);
    void addDescriptors(cv::Mat descs);
    void addPoints(std::vector<cv::Point3f> p3d);
    void removeDuplicates ();
    void filterNaNKeyPoints (cv::Mat depth, std::vector<cv::KeyPoint> kpts,
								std::vector<cv::KeyPoint> &filtered_kpts,
								std::vector<cv::Point3f> &filtered_p3d);

    // Model
    std::vector<cv::Point3f> p3d_;
    cv::Mat descs_;

    Pipeline2D pipe2d_;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
};
