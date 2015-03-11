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
    vector<Vec3b> color;
    for ( size_t i = 0; i < images.size(); ++i ) {
        Mat rectified_image;
        rectifyImage (images[i], corners2d[i], dimensions,
                      rectified_image);

        /*
        std::stringstream ss;
        ss << i;
        imshow ("Rectified Image", rectified_image); waitKey(0);
        string filename = "rectified_image_" + ss.str() + ".png";
        imwrite(filename.c_str() , rectified_image);
        */

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

        saveColor(rectified_image, keypoints, color);
    }
    // for visualization
    vector<Point3f> points3d;
    object.getPoints (points3d);
    savePoints3d (points3d, color, "model.pcd");
    //cout << "Total num p3ds: " << points3d.size() << endl;

    return object;
}

void POM::savePoints3d (std::vector<cv::Point3f> points3d, std::vector<Vec3b> bgrs, std::string filename) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  for (size_t i = 0; i < points3d.size(); ++i) {
    pcl::PointXYZRGB pt;
    pt.x = points3d[i].x;
    pt.y = points3d[i].y;
    pt.z = points3d[i].z;
    int32_t rgb = (bgrs[i][2] << 16) | (bgrs[i][1] << 8) | bgrs[i][0];
    pt.rgb = *(float *)(&rgb);
    cloud->push_back(pt);
  }

  cloud->width = points3d.size ();
  cloud->height = 1;

  pcl::io::savePCDFileASCII(filename, *cloud);
}

void POM::saveColor(Mat image, std::vector<KeyPoint> kpts, vector<Vec3b> &bgrs) {
    for (size_t i = 0; i < kpts.size(); ++i) {
        Vec3b bgr = image.at<Vec3b>(kpts[i].pt.y, kpts[i].pt.x);
        bgrs.push_back(bgr);
    }

}
////////////////////////////////////////////////////////////////////////////////
//  PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
void POM::extractFeatures (Mat image, vector<KeyPoint> &keypoints, Mat &descriptors) {
    Mat gray;
    pipeline2d_.getGray (image, gray);
    pipeline2d_.extractFeatures(gray, USE_SIFTGPU, keypoints, descriptors);
}

void POM::rectifyImage (Mat image, vector<Point2f> corners2d, Point3f dimensions,
                    Mat &rectified_image) {
    vector<Point2f> frontal_corners = getCorners (corners2d);
    //cout << corners2d << endl << frontal_corners << endl;
    Mat T = getPerspectiveTransform (corners2d, frontal_corners);

    vector<Point2f> warped_corners;
    cv::perspectiveTransform(corners2d, warped_corners, T);
    //cout << T << endl;
    Rect bb = boundingRect(warped_corners);
    cv::Size size (bb.width, bb.height);
    //cout << size << endl;
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
        //cout << points3d[i].x << " " << points3d[i].y << " " << points3d[i].z << endl;
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
	Mat Rz = Mat::eye(3, 3, CV_32F);
    Rz.at<float>(0,0) = 0;
    Rz.at<float>(1,1) = 0;
    Rz.at<float>(0,1) = 1;
    Rz.at<float>(1,0) = -1;
	Mat Ry = Mat::eye(3, 3, CV_32F);
    Ry.at<float>(0,0) = 0;
    Ry.at<float>(2,2) = 0;
    Ry.at<float>(0,2) = -1;
    Ry.at<float>(2,0) = 1;
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
    R = R * Ry * Rz;
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
        if ( face == 0 || face == 3) // front or back
            points3d[i].y = (keypoints[i].pt.x - image.cols/2) * dimensions.x / image.cols;
        else
            points3d[i].y = (keypoints[i].pt.x - image.cols/2) * dimensions.z / image.cols;
        points3d[i].z = -(keypoints[i].pt.y - image.rows/2) * dimensions.y / image.rows;
    }
}

void POM::convert2Dto3Dcyl (int face, Mat image, Point3f dimensions, vector<KeyPoint> keypoints,
                            vector<Point3f> &points3d) {
    points3d.resize(keypoints.size());
    double H = dimensions.y;
    double R = dimensions.x; // radius

    double fx = 740 * 640 / image.cols;
    double fy = 740 * 480 / image.rows;
    //cout << fx << " " << fy << endl;
    double scaleX = 2 * R / image.cols;
    double scaleY = H / image.rows;
    //cout << scaleX << " " << scaleY << endl;
    double sol1, sol2;
    for (size_t i = 0; i < keypoints.size(); ++i) {
        double du = keypoints[i].pt.x - (image.cols / 2);
        double dv = keypoints[i].pt.y - (image.rows / 2);
        solveSndDegEq( (fx * fx) / (du * du) + 1,
                       2 * scaleX * (fx * fx) / du,
                       scaleX * scaleX * fx * fx - R * R,
                       sol1, sol2);
        //cout << keypoints[i].pt.x << ": " << sol1 << " " << sol2 << endl;
        if (du > 0)
            points3d[i].x = max(sol1, sol2);
        else
            points3d[i].x = min(sol1, sol2);
        //points3d[i].x = sol2;
        points3d[i].z = sqrt(R * R - points3d[i].x * points3d[i].x);
        points3d[i].y = ((points3d[i].z / fy) - scaleX) * dv;
    }
    //points3d[i].y = (keypoints[i].pt.x - image.cols / 2) * D / image.cols;
    //points3d[i].x = sqrt(R * R - points3d[i].y * points3d[i].y);
    //points3d[i].z = -(keypoints[i].pt.y - image.rows / 2) * H / image.rows;
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

int POM::solveSndDegEq(double a, double b, double c, double &sol1, double &sol2) {
    double delta = b * b - 4 * a * c;
    if (delta<0) {
        cout<< "Delta " << " == " << delta << ": solution is not an integer" << endl;
        return 1;
    } else {
        sol1=(-b+sqrt(delta))/(2*a);
        sol2=(-b-sqrt(delta))/(2*a);
    }
    return 0;
}
