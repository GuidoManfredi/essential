#include "ImageSynthesizer.h"

using namespace std;
using namespace cv;

void ImageSynthesizer::views (cv::Mat img, std::vector<cv::Point2f> corners,
                              std::vector<Mat> &views,
                              std::vector< std::vector<cv::Point2f> > &warped_corners) {
  std::vector<cv::Point2f> frontal_corners;
  Mat frontal;
  // Special case of frontal view, starting point to generate all other images
  generateFrontalView (img, corners,
                       frontal, frontal_corners);
  views.push_back (frontal);
  warped_corners.push_back (frontal_corners);
  // rest of views
  Point3f yaw(0, 20, 180), pitch(0, 15, 60), roll(0.0, 2.0, 1.0), scale(0, 0.5, 1.0);
  //Point3f yaw(0, 2.0, 1.0), pitch(0, 2.0, 1.0), roll(-80, 20, 80), scale(0, 0.5, 1.0);
  //Point3f yaw(0, 2.0, 1.0), pitch(-80, 20, 80), roll(0.0, 2.0, 1.0), scale(0, 0.5, 1.0);
  //generateViews (frontal, yaw, pitch, roll, scale, views, corners);
  Point3f theta(0, 15, 60);
  generateViewsCollet (frontal, frontal_corners, theta, views, warped_corners);
}

void ImageSynthesizer::generateFrontalView (cv::Mat img, std::vector<cv::Point2f> in_corners,
                                            cv::Mat &frontal,
                                            std::vector<cv::Point2f> &frontal_corners)
{
  const float w = img.cols;
  const float h = img.rows;
  // Corners in rectified image
  frontal_corners.push_back (Point2f (0.0f, 0.0f));
  frontal_corners.push_back (Point2f (w, 0.0f));
  frontal_corners.push_back (Point2f (w, h));
  frontal_corners.push_back (Point2f (0.0f, h));

  Mat T = getPerspectiveTransform (in_corners, frontal_corners);
  warpPerspective (img, frontal, T, Size(w, h));
}

void ImageSynthesizer::generateViews (const cv::Mat img,
                                        cv::Point3f yaw, cv::Point3f pitch, cv::Point3f roll, cv::Point3f scale,
                                        std::vector<Mat> &warped_img,
                                        std::vector< std::vector<cv::Point2f> > &warped_corners)
{
  //cv::namedWindow("Synthetic View");
  //warped_img.clear ();
  //warped_corners.clear ();
  // Make a copy of img, with larger borders, to cater for rotation
  int size = sqrt(img.cols*img.cols + img.rows*img.rows);
  cv::Mat big = cv::Mat(cv::Size(size,size), img.type());
  // Fill the background image with mid level img
  cv::rectangle(big, cv::Point(0,0), cv::Point(big.cols-1, big.rows-1), CV_RGB(128,128,128), CV_FILLED);
  // Offset, so the image is positioned in the middle
  int xstart = (big.cols - img.cols)/2;
  int ystart = (big.rows - img.rows)/2;
  cv::Mat sub = big(cv::Range(ystart, ystart + img.rows), cv::Range(xstart, xstart + img.cols));
  img.copyTo(sub);

  std::vector<Point2f> corners;
  corners.push_back (Point2f(xstart, ystart));
  corners.push_back (Point2f(xstart + img.cols, ystart));
  corners.push_back (Point2f(xstart + img.cols, ystart + img.rows));
  corners.push_back (Point2f(xstart, ystart + img.rows));

  int count = 0;
  const int total_views = (scale.z-scale.x)
                         * ((yaw.z-yaw.x)/yaw.y+1)
                         * ((pitch.z-pitch.x)/pitch.y+1);
                         //* ((roll.z-roll.x)/roll.y);
  for(int s=scale.x; s < scale.z; ++s) {
        cv::Mat resized, tmp_affine;
        float resize_factor = pow(scale.y, (float)s);
        std::vector<Point2f> resized_corners(4);
        resized_corners[0] = corners[0] * resize_factor;
        resized_corners[1] = corners[1] * resize_factor;
        resized_corners[2] = corners[2] * resize_factor;
        resized_corners[3] = corners[3] * resize_factor;
        cv::resize(big, resized, cv::Size(big.cols*resize_factor, big.rows*resize_factor));

        for(int y=yaw.x; y <= yaw.z; y+=yaw.y) {
            for(int p=pitch.x; p <= pitch.z; p+=pitch.y) {
                for(int r=roll.x; r < roll.z; r+=roll.y) {
                    Mat warped;
                    warpImage(resized, warped, y*M_PI/180.0, p*M_PI/180.0, r*M_PI/180.0, tmp_affine);
                    warped_img.push_back (warped);
                    /*
                    imshow("View", warped);
                    waitKey(0);
                    */
/*
                    char path[50];
                    sprintf (path, "view%d.png", count);
                    imwrite (path, warped);
*/
                    std::vector<Point2f> tmp_warped_corners;
                    cv::transform(resized_corners, tmp_warped_corners, tmp_affine);

                    warped_corners.push_back (tmp_warped_corners);
                    ++count;
                }
            }
        }
    }
}

void ImageSynthesizer::generateViewsCollet (const cv::Mat img,
                                              std::vector<cv::Point2f> corners,
                                              cv::Point3f theta,
                                              std::vector<Mat> &warped_img,
                                              std::vector< std::vector<cv::Point2f> > &warped_corners)
{
  int count = 0;
  float b = 72;
  for (int l=theta.x; l<=theta.z; l+=theta.y) {
        cv::Mat tmp_affine;
        float t = 1/cosf(l*M_PI/180);
        for(int phi=0; phi<180 ; phi+=b/t) {
            Mat tmp_warped_img;
            std::vector<cv::Point2f> tmp_warped_corners;
            warpImageCollet(img, tmp_warped_img, corners, tmp_warped_corners, l*M_PI/180, phi*M_PI/180, 0*M_PI/180, tmp_affine);
            warped_img.push_back (tmp_warped_img);
            warped_corners.push_back (tmp_warped_corners);
            /*
            imshow("View", warped);
            waitKey(0);
            */
/*
            char path[50];
            sprintf (path, "view%d.png", count);
            imwrite (path, warped);
            */
            ++count;
        }
    }
}

void ImageSynthesizer::warpImage(const cv::Mat &in, cv::Mat &out, float yaw, float pitch, float roll, cv::Mat &affine_out)
{
    cv::Mat to_centre = cv::Mat::eye(4,4,CV_32F);
    cv::Mat to_origin = cv::Mat::eye(4,4,CV_32F);

    to_origin.at<float>(0,3) = -in.cols/2;
    to_origin.at<float>(1,3) = -in.rows/2;

    to_centre.at<float>(0,3) = in.cols/2;
    to_centre.at<float>(1,3) = in.rows/2;

    cv::Mat R = makeRotation(pitch, yaw, roll);
    cv::Mat T = to_centre*R*to_origin;
    // We let the 3D point to have z=0
    cv::Mat affine(2,3,CV_32F);

    affine.at<float>(0,0) = T.at<float>(0,0);
    affine.at<float>(0,1) = T.at<float>(0,1);
    affine.at<float>(0,2) = T.at<float>(0,3);
    affine.at<float>(1,0) = T.at<float>(1,0);
    affine.at<float>(1,1) = T.at<float>(1,1);
    affine.at<float>(1,2) = T.at<float>(1,3);
    //cout << affine << endl;

    affine_out = affine;
    cv::warpAffine(in, out, affine, cv::Size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(128,128,128));
}

void ImageSynthesizer::warpImageCollet(const cv::Mat &in, cv::Mat &out, const std::vector<Point2f> &corners, std::vector<Point2f> &warped_corners, float theta, float phi, float psi, Mat &affine_out)
{
    float lambda = 1.0;
    Mat A = makeAffine(phi, theta, psi, lambda);

    Mat h_A = Mat::eye(3, 3, CV_32F);
    Mat tmp = h_A (cv::Rect(0,0,2,2));
    A.copyTo(tmp);
    // Warp the corners in to find size of the destination image
    cv::transform(corners, warped_corners, A);
    Rect bb = boundingRect(warped_corners);
    cv::Size size (bb.width, bb.height);

    Mat centre = Mat::eye(3, 3, CV_32F);
    centre.at<float>(0, 2) = bb.width/2;
    centre.at<float>(1, 2) = bb.height/2;
    Mat origin = Mat::eye(3, 3, CV_32F);
    origin.at<float>(0, 2) = -in.cols/2;
    origin.at<float>(1, 2) = -in.rows/2;
    /*
    Mat centre = Mat::eye(3, 3, CV_32F);
    centre.at<float>(0, 2) = in.cols/2;
    centre.at<float>(1, 2) = in.rows/2;
    Mat origin = Mat::eye(3, 3, CV_32F);
    origin.at<float>(0, 2) = -in.cols/2;
    origin.at<float>(1, 2) = -in.rows/2;
    */

    Mat h_affine = centre*h_A*origin;
    Mat affine = h_affine (Rect(0,0,3,2)).clone();
    //cout << affine << endl;

    affine_out = affine;

    cv::warpAffine(in, out, affine, size, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(128,128,128));
    cv::transform(corners, warped_corners, affine);
}

cv::Mat ImageSynthesizer::makeRotation(float x, float y, float z)
{
    cv::Mat X = cv::Mat::eye(4,4,CV_32F);
    cv::Mat Y = cv::Mat::eye(4,4,CV_32F);
    cv::Mat Z = cv::Mat::eye(4,4,CV_32F);

    float sinx = sinf(x);
    float siny = sinf(y);
    float sinz = sinf(z);

    float cosx = cosf(x);
    float cosy = cosf(y);
    float cosz = cosf(z);

    X.at<float>(1,1) = cosx;
    X.at<float>(1,2) = -sinx;
    X.at<float>(2,1) = sinx;
    X.at<float>(2,2) = cosx;

    Y.at<float>(0,0) = cosy;
    Y.at<float>(0,2) = siny;
    Y.at<float>(2,0) = -siny;
    Y.at<float>(2,2) = cosy;

    Z.at<float>(0,0) = cosz;
    Z.at<float>(0,1) = -sinz;
    Z.at<float>(1,0) = sinz;
    Z.at<float>(1,1) = cosz;

	cv::Mat R = Z*Y*X;

	return R;
}

cv::Mat ImageSynthesizer::makeAffine(float phi, float theta, float psi, float lambda) {
    Mat A;
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

    A = lambda*Rpsi*T*Rphi;
    return A;
}
