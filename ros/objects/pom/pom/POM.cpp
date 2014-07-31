#include "POM.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

POM::POM () {}

void POM::setIntrinsic(Mat K) {
    K.copyTo(K_);
    if (!K_.data)
        cout << "Set intrinsic failed" << endl;
}

void POM::loadIntrinsic (string calibration_file) {
    Mat K(3,3,CV_32F);
    FileStorage r_fs;
    r_fs.open (calibration_file, cv::FileStorage::READ);
    r_fs["camera_matrix"]>>K;
    r_fs.release ();
    setIntrinsic (K);
}

Object POM::model (std::vector<cv::Mat> images,
                   std::vector<std::vector<Point2f> > corners2d,
                   Point3f dimensions) {
    Object object;
    for ( size_t i = 0; i < images.size(); ++i ) {
        Mat rectified_image;
        rectifyImage (i, images[i], corners2d[i], dimensions,
                      rectified_image);
        //imshow ("Rectified Image", rectified_image); waitKey(0);

        vector<KeyPoint> keypoints;
        Mat descriptors;
        vector<Point3f> points3d;
        extractFeatures (rectified_image, keypoints, descriptors);
        cout << "Extracted " << keypoints.size() << " keypoints." << endl;

        convert2Dto3D (i, rectified_image, dimensions, corners2d[i], keypoints,
                       points3d);
        cout << "Computed " << points3d.size() << " 3D points." << endl;
        object.addView (points3d, descriptors);
    }
    vector<Point3f> points3d;
    object.getPoints (points3d);
    //savePoints3d (points3d, "output.pcd");

    return object;
}
/*
void POM::savePoints3d (std::vector<cv::Point3f> points3d, std::string filename) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  for (size_t i = 0; i < points3d.size(); ++i) {
    pcl::PointXYZ pt;
    pt.x = points3d[i].x;
    pt.y = points3d[i].y;
    pt.z = points3d[i].z;
    cloud->push_back(pt);
  }

  cloud->width = points3d.size ();
  cloud->height = 1;

  pcl::io::savePCDFileASCII(filename, *cloud);
}
*/
////////////////////////////////////////////////////////////////////////////////
//  PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
void POM::extractFeatures (Mat image, vector<KeyPoint> &keypoints, Mat &descriptors) {
    Mat gray;
    pipeline2d_.getGray (image, gray);
    pipeline2d_.extractFeatures(gray, keypoints, descriptors);
}

void POM::rectifyImage (int face, Mat image, vector<Point2f> corners2d, Point3f dimensions,
                    Mat &rectified_image) {
    vector<Point2f> frontal_corners = getCorners (corners2d);
    //cout << corners2d << endl << frontal_corners << endl;
    Mat T = getPerspectiveTransform (corners2d, frontal_corners);

    vector<Point2f> warped_corners;
    cv::perspectiveTransform(corners2d, warped_corners, T);
    Rect bb = boundingRect(warped_corners);
    cv::Size size (bb.width, bb.height);

    warpPerspective(image, rectified_image, T, size);
}

void POM::convert2Dto3D(int face, Mat image, Point3f dimensions, vector<Point2f> corners2d,
                         vector<KeyPoint> keypoints,
                         vector<Point3f> &points3d) {
    points3d.resize(keypoints.size());
    Mat R, t;
    vector<Point3f> local_points3d;
    local2global (face, dimensions,
                  R, t);
    convert2DtoLocal3D (face, image, dimensions, keypoints, local_points3d);
    for (size_t i = 0; i < points3d.size(); ++i) {
        points3d[i].x = local_points3d[i].x * R.at<float>(0, 0)
                      + local_points3d[i].y * R.at<float>(0, 1)
                      + local_points3d[i].z * R.at<float>(0, 2)
                      + t.at<float>(0);
        points3d[i].y = local_points3d[i].x * R.at<float>(1, 0)
                      + local_points3d[i].y * R.at<float>(1, 1)
                      + local_points3d[i].z * R.at<float>(1, 2)
                      + t.at<float>(1);
        points3d[i].z = local_points3d[i].x * R.at<float>(2, 0)
                      + local_points3d[i].y * R.at<float>(2, 1)
                      + local_points3d[i].z * R.at<float>(2, 2)
                      + t.at<float>(2);
    }
}

void POM::local2global (int face, Point3f dimensions,
                         Mat &R, Mat &t) {
    t = Mat::zeros(3, 1, CV_32F);
	R = Mat::eye(3, 3, CV_32F);
    if ( face == 0 ) {
        t.at<float>(0) = dimensions.z/2;
    } else if ( face == 1 ) {
        t.at<float>(1) = dimensions.x/2;
        R.at<float>(0,0) = 0;
        R.at<float>(1,1) = 0;
        R.at<float>(0,1) = -1;
        R.at<float>(1,0) = 1;
    } else if ( face == 2 ) {
        t.at<float>(1) = -dimensions.x/2;
        R.at<float>(0,0) = 0;
        R.at<float>(1,1) = 0;
        R.at<float>(0,1) = 1;
        R.at<float>(1,0) = -1;
    } else if ( face == 3 ) {
        t.at<float>(0) = -dimensions.z/2;
        R.at<float>(0,0) = -1;
        R.at<float>(1,1) = -1;
    } else if ( face == 4 ) {
        ;
    } else {
        ;
    }
}

void POM::convert2DtoLocal3D (int face, Mat image, Point3f dimensions, vector<KeyPoint> keypoints,
                          vector<Point3f> &points3d) {
    points3d.resize(keypoints.size());
    for (size_t i = 0; i < keypoints.size(); ++i) {
        points3d[i].x = 0;
        if ( face == 0 || face == 2)
            points3d[i].y = (keypoints[i].pt.x - image.cols/2) * dimensions.x/image.cols;
        else
            points3d[i].y = (keypoints[i].pt.x - image.cols/2) * dimensions.z/image.cols;
        points3d[i].z = -(keypoints[i].pt.y - image.rows/2) * dimensions.y/image.rows;
    }
}

vector<Point2f> POM::getCorners (vector<Point2f> corners2d) {
    vector<Point2f> corners;
    Rect bb = boundingRect(corners2d);
    corners.push_back (Point2f(0, 0));
    corners.push_back (Point2f(bb.width-1, 0));
    corners.push_back (Point2f(bb.width-1, bb.height-1));
    corners.push_back (Point2f(0, bb.height-1));
    return corners;
}
