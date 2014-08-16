#include "tools.h"

void P2Rt (cv::Mat P,
            cv::Mat &R, cv::Mat &t) {
    R = P(cv::Rect(0,0,3,3));
    t = P(cv::Rect(0,3,4,1));
}

void vec2pcd (std::vector<Point3f> p3d, pcl::PointCloud<pcl::PointXYZ>::Ptr pcd) {
    pcd->width = p3d.size();
    pcd->height = 1;
    pcd->is_dense = false;
    pcd->points.resize (pcd->width * pcd->height);
    for (size_t i = 0; i < pcd->points.size (); ++i) {
        pcd->points[i].x = p3d[i].x;
        pcd->points[i].y = p3d[i].y;
        pcd->points[i].z = p3d[i].z;
    }
}
