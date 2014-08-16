#pragma once

#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>
#include <tf/transform_datatypes.h>

void P2Rt (cv::Mat P, cv::Mat &R, cv::Mat &t);
void vec2pcd (std::vector<cv::Point3f> p3d, pcl::PointCloud<pcl::PointXYZ>::Ptr pcd);
cv::Mat transform2mat (tf::StampedTransform transform);

