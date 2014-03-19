#include <opencv2/imgproc/imgproc.hpp>

#include "SyntheticViewGenerator.h"

using namespace std;
using namespace cv;

SyntheticViewGenerator::SyntheticViewGenerator () {
    b_ = 72;
}

int SyntheticViewGenerator::generateViews (Mat image, vector<Point2f> corners,
                                            std::vector<Mat> &views, std::vector<Mat> &affines) {
    views.clear();

    Mat frontal = generateFrontalView (image, corners);
    views.push_back (frontal);

    Point3f theta(0, 15, 60);
    generateAffineViews (frontal, theta, views, affines);

    return views.size();
}

Mat SyntheticViewGenerator::generateFrontalView (cv::Mat image, vector<Point2f> corners)
{
    Rect bb = boundingRect(corners);
    cv::Size size (bb.width, bb.height);
    vector<Point2f> frontal_corners = getCorners (size.width, size.height);

    Mat T = getPerspectiveTransform (corners, frontal_corners);

    Mat frontal;
    warpPerspective (image, frontal, T, size);

    return frontal;
}

void SyntheticViewGenerator::generateAffineViews (Mat image, Point3f theta,
                                                  vector<Mat> &views, vector<Mat> &affines) {
  for (int l=theta.x; l<=theta.z; l+=theta.y) {
        float t = 1/cosf(l*M_PI/180);
        for(int phi=0; phi<180 ; phi+=b_/t) {
            Mat affine_transform;
            Mat affine_view = generateAffineView (image, l*M_PI/180, phi*M_PI/180, 0*M_PI/180, affine_transform);
            views.push_back (affine_view);
            affines.push_back (affine_transform);
        }
    }
}

Mat SyntheticViewGenerator::generateAffineView (const cv::Mat image, float theta, float phi, float psi,
                                                Mat &affine) {
    vector<Point2f> frontal_corners = getCorners (image);
    Mat warped_image = warpImage (image, frontal_corners, theta, phi, psi, affine);
    return warped_image;
}

Mat SyntheticViewGenerator::warpImage(const cv::Mat image, const std::vector<Point2f> corners, float theta, float phi, float psi,
                                      Mat &affine3x2)
{
    // Rotation part
    float lambda = 1.0;
    Mat rotation_A = makeAffine(phi, theta, psi, lambda); // A is a 2x2 matrix
    Mat A = Mat::eye(3, 3, CV_32F);
    Mat upper_left_A = A (cv::Rect(0,0,2,2));
    rotation_A.copyTo(upper_left_A);
    // Warp the corners to find size of the destination image
    float w = image.cols;
    float h = image.rows;
    vector<Point2f> warped_corners;
    cv::transform(corners, warped_corners, rotation_A);
    Rect bb = boundingRect(warped_corners);
    cv::Size size (bb.width, bb.height);
    // Find the translation part
    Mat centre = Mat::eye(3, 3, CV_32F);
    centre.at<float>(0, 2) = bb.width/2;
    centre.at<float>(1, 2) = bb.height/2;
    Mat origin = Mat::eye(3, 3, CV_32F);
    origin.at<float>(0, 2) = -image.cols/2;
    origin.at<float>(1, 2) = -image.rows/2;
    Mat affine3x3 = centre*A*origin;
    affine3x2 = affine3x3 (Rect(0,0,3,2)).clone();

    Mat warped_image;
    cv::warpAffine(image, warped_image, affine3x2, size, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    return warped_image;
}

cv::Mat SyntheticViewGenerator::makeAffine(float phi, float theta, float psi, float lambda) {
    float t = 1/cosf(theta);

    float sin_phi = sinf(phi);
    float cos_phi = cosf(phi);

    float sin_psi = sinf(psi);
    float cos_psi = cosf(psi);

    Mat Rpsi = cv::Mat::eye(2,2,CV_32F);
    Rpsi.at<float>(0,0) = cos_psi;    Rpsi.at<float>(0,1) = -sin_psi;
    Rpsi.at<float>(1,0) = sin_psi;    Rpsi.at<float>(1,1) = cos_psi;

    Mat Rphi = cv::Mat::eye(2,2,CV_32F);
    Rphi.at<float>(0,0) = cos_phi;    Rphi.at<float>(0,1) = -sin_phi;
    Rphi.at<float>(1,0) = sin_phi;    Rphi.at<float>(1,1) = cos_phi;

    Mat T = cv::Mat::eye(2,2,CV_32F);
    T.at<float>(0, 0) = t;

    Mat A = lambda*Rpsi*T*Rphi;
    return A;
}

std::vector<Point2f> SyntheticViewGenerator::getCorners (Mat image) {
    const float w = image.cols;
    const float h = image.rows;
    vector<Point2f> frontal_corners;
    frontal_corners.push_back (Point2f (0.0f, 0.0f));
    frontal_corners.push_back (Point2f (w, 0.0f));
    frontal_corners.push_back (Point2f (w, h));
    frontal_corners.push_back (Point2f (0.0f, h));
    return frontal_corners;
}

std::vector<Point2f> SyntheticViewGenerator::getCorners (float width, float height) {
    vector<Point2f> frontal_corners;
    frontal_corners.push_back (Point2f (0.0f, 0.0f));
    frontal_corners.push_back (Point2f (width, 0.0f));
    frontal_corners.push_back (Point2f (width, height));
    frontal_corners.push_back (Point2f (0.0f, height));
    return frontal_corners;
}
