#include "POM.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

POM::POM () {}

Object POM::model (SHAPE shape, std::vector<cv::Mat> images, std::vector<int> faces,
                   std::vector<std::vector<Point2f> > corners2d,
                   Point3f dimensions) {
    Object object;
    for ( size_t i = 0; i < images.size(); ++i ) {
        Mat rectified_image;
        rectifyImage (images[i], corners2d[i], dimensions,
                      rectified_image);
        //imshow ("Rectified Image", rectified_image); waitKey(0);

        vector<KeyPoint> keypoints;
        Mat descriptors;
        vector<Point3f> points3d;
        extractFeatures (rectified_image, keypoints, descriptors);
        cout << "Extracted " << keypoints.size() << " keypoints." << endl;
        //cout << "face: " << faces[i] << endl;
        convert2Dto3D (shape, faces[i], rectified_image, dimensions, corners2d[i], keypoints,
                       points3d);
        cout << "Computed " << points3d.size() << " 3D points." << endl;
        object.addView (points3d, descriptors);
    }
    // for visualization
    vector<Point3f> points3d;
    object.getPoints (points3d);
    savePoints3d (points3d, "model.pcd");
    //cout << "Total num p3ds: " << points3d.size() << endl;

    return object;
}

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

////////////////////////////////////////////////////////////////////////////////
//  PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
void POM::extractFeatures (Mat image, vector<KeyPoint> &keypoints, Mat &descriptors) {
    Mat gray;
    pipeline2d_.getGray (image, gray);
    pipeline2d_.extractFeatures(gray, keypoints, descriptors);
}

void POM::rectifyImage (Mat image, vector<Point2f> corners2d, Point3f dimensions,
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

void POM::convert2Dto3D(SHAPE shape, int face, Mat image, Point3f dimensions, vector<Point2f> corners2d,
                         vector<KeyPoint> keypoints,
                         vector<Point3f> &points3d) {
    points3d.resize(keypoints.size());
    Mat R, t;
    vector<Point3f> local_points3d;
    local2global (shape, face, dimensions,
                  R, t);
    convert2DtoLocal3D (shape, face, image, dimensions, keypoints, local_points3d);
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

void POM::local2global (SHAPE shape, int face, Point3f dimensions,
                         Mat &R, Mat &t) {
    if (shape == PAVE) {
        local2globalPave(face, dimensions, R, t);
    } else if (shape == CYL) {
        local2globalCyl(face, dimensions, R, t);
    } else {
        cout << "POM.cpp: local2global: Unknown shape type " << shape << endl;
    }
}

void POM::local2globalPave (int face, Point3f dimensions,
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

void POM::local2globalCyl (int face, Point3f dimensions,
                            Mat &R, Mat &t) {
    t = Mat::zeros(3, 1, CV_32F);
	R = Mat::eye(3, 3, CV_32F);
    if ( face == 0 ) {
        t.at<float>(0) = 0; // do nothing
    } else if ( face == 1 ) {
        t.at<float>(1) = 0;
        R.at<float>(0,0) = 0;
        R.at<float>(1,1) = 0;
        R.at<float>(0,1) = -1;
        R.at<float>(1,0) = 1;
    } else if ( face == 2 ) {
        t.at<float>(1) = 0;
        R.at<float>(0,0) = 0;
        R.at<float>(1,1) = 0;
        R.at<float>(0,1) = 1;
        R.at<float>(1,0) = -1;
    } else if ( face == 3 ) {
        t.at<float>(0) = 0;
        R.at<float>(0,0) = -1;
        R.at<float>(1,1) = -1;
    } else if ( face == 4 ) {
        ;
    } else {
        ;
    }
}

void POM::convert2DtoLocal3D (SHAPE shape, int face, Mat image, Point3f dimensions, vector<KeyPoint> keypoints,
                                vector<Point3f> &points3d) {
    if (shape == PAVE) {
        convert2Dto3Dpave(face, image, dimensions, keypoints, points3d);
    } else if (shape == CYL) {
        convert2Dto3Dcyl(face, image, dimensions, keypoints, points3d);
    } else {
        cout << "POM.cpp: convert2DtoLocal3D: Unknown shape type " << shape << endl;
    }
}

void POM::convert2Dto3Dpave (int face, Mat image, Point3f dimensions, vector<KeyPoint> keypoints,
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

void POM::convert2Dto3Dcyl (int face, Mat image, Point3f dimensions, vector<KeyPoint> keypoints,
                            vector<Point3f> &points3d) {
    points3d.resize(keypoints.size());
    float diameter = 2 * dimensions.x; // 2 * radius
    for (size_t i = 0; i < keypoints.size(); ++i) {
        points3d[i].y = (keypoints[i].pt.x - image.cols/2) * diameter/image.cols;
        points3d[i].x = sqrt(dimensions.x * dimensions.x - points3d[i].y * points3d[i].y);
        points3d[i].z = -(keypoints[i].pt.y - image.rows/2) * dimensions.y/image.rows;
        //cout << points3d[i].x << " " << points3d[i].y << endl;
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

