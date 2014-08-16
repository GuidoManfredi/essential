#include "tools.h"

using namespace cv;

void P2Rt (cv::Mat P,
            cv::Mat &R, cv::Mat &t) {
    R = P(cv::Rect(0,0,3,3));
    t = P(cv::Rect(0,3,4,1));
}

void vec2pcd (std::vector<Point3f> p3d, pcl::PointCloud<pcl::PointXYZ>::Ptr pcd) {
    pcd->clear();
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

cv::Mat transform2mat (tf::StampedTransform transform) {
    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    double z = transform.getOrigin().z();
    tf::Matrix3x3 R(transform.getRotation());
    Mat P = (Mat_<float>(4,4) << R[0][0], R[0][1], R[0][2], x,
                                 R[1][0], R[1][1], R[1][2], y, 
                                 R[2][0], R[2][1], R[2][2], z,
                                 0, 0, 0, 1);
    return P;    
}
