#include "POM.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

POM::POM () {}

void POM::setIntrinsic(Mat K) {
    K.copyTo(K_);
}

Object POM::model (std::vector<cv::Mat> images,
                   std::vector<std::vector<Point2f> > corners2d,
                   Point3f dimensions) {
    Object object;
    for ( size_t i = 0; i < images.size(); ++i ) {
        vector<KeyPoint> keypoints;
        Mat descriptors;
        vector<Point3f> points3d;
        extractFeatures (images[i], keypoints, descriptors);
        Mat rectified_image;
        rectifyImage (i, images[i], corners2d[i], dimensions,
                      rectified_image);
//        imshow ("Kikou", rectified_image);
//        waitKey(0);
        convert2Dto3D (i, images[i], dimensions, corners2d[i], keypoints,
                       points3d);
        savePoints3d (points3d, "test.pcd");
        cout << i << endl << Mat(points3d) << endl;
        //object.addView (points3d, descriptors);
    }
    return object;
}

void savePoints3d (std::vector<cv::Point3f> points3d, std::string filename) {
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
    vector<Point2f> frontal_corners = getCorners (image);
//    cout << corners2d << endl << frontal_corners << endl;
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
    convert2DtoLocal3D (image, dimensions, keypoints, local_points3d);
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
        t.at<float>(0) = -dimensions.z/2;
        R.at<float>(0,0) = -1;
        R.at<float>(1,1) = -1;
    } else {
        t.at<float>(1) = -dimensions.x/2;
        R.at<float>(0,0) = 0;
        R.at<float>(1,1) = 0;
        R.at<float>(0,1) = 1;
        R.at<float>(1,0) = -1;
    }
}

//p3d.dim = keypoints.pt.dim * object.dim/image.dim
void POM::convert2DtoLocal3D (Mat image, Point3f dimensions, vector<KeyPoint> keypoints,
                          vector<Point3f> &points3d) {
    points3d.resize(keypoints.size());
    for (size_t i = 0; i < keypoints.size(); ++i) {
        points3d[i].x = 0;
        points3d[i].y = keypoints[i].pt.x * dimensions.x/image.cols;
        points3d[i].z = -keypoints[i].pt.y * dimensions.y/image.rows;
    }
}

vector<Point3f> POM::getLocalCorners3DPaveDroit (int face, Point3f dimensions) {
    float w, h, d; // on utilise des demi mesures car repere au centre de la face.
    w = dimensions.x/2;
    h = dimensions.y/2;
    d = dimensions.z/2;

    vector<Point3f> points3d;
    Point3f ul, ur, dl, dr; // up, down, right, left. ul = up left
    if ( face == 0 || face == 2 ) {
        ul = Point3f (0, -w, h);
        ur = Point3f (0, w, h);
        dl = Point3f (0, -w, -h);
        dr = Point3f (0, w, -h);
    } else {
        ul = Point3f (0, -d, h);
        ur = Point3f (0, d, h);
        dl = Point3f (0, -d, -h);
        dr = Point3f (0, d, -h);
    }
    points3d.push_back (ul);
    points3d.push_back (ur);
    points3d.push_back (dl);
    points3d.push_back (dr);
    return points3d;
}

vector<Point2f> POM::getCorners (Mat image) {
    std::vector<Point2f> corners;
    corners.push_back (Point2f(0, 0));
    corners.push_back (Point2f(image.cols, 0));
    corners.push_back (Point2f(image.cols, image.rows));
    corners.push_back (Point2f(0, image.rows));
    return corners;
}
