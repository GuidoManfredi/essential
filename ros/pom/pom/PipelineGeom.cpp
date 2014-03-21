#include <opencv2/calib3d/calib3d.hpp>

#include "PipelineGeom.h"

using namespace std;
using namespace cv;

PipelineGeom::PipelineGeom (string calibration_file) {
	Mat K(3,3,CV_32F);
    Mat d(5,1,CV_32F);
    FileStorage r_fs;
    r_fs.open (calibration_file, cv::FileStorage::READ);
    r_fs["camera_matrix"]>>K;
    r_fs["distortion_coefficients"]>>d;
    r_fs.release ();
	K_ = K;
	d_ = d;
}

Mat PipelineGeom::filterMatchesHomography (vector<KeyPoint> query_keypoints, vector<KeyPoint> train_keypoints,
                                            vector<DMatch> &matches) {
    Mat H;
    if ( matches.size() < 4 ) {
        H = Mat::eye (3, 3, CV_32F);
        return H;
    }

    std::vector<Point2f> query_pts, train_pts;
    match2points(query_keypoints, train_keypoints, matches, query_pts, train_pts);

    vector<unsigned char> mask(query_pts.size());
    H = cv::findHomography(train_pts, query_pts, CV_RANSAC, 3.0, mask);
    //homography = cv::findHomography(pts1, pts2, CV_RANSAC, 0.5, mask);

    vector<DMatch> inliers;
    for (size_t i=0; i<mask.size(); i++) {
        if (mask[i])
            inliers.push_back(matches[i]);
    }
    matches.swap(inliers);
    return H;
}

Mat PipelineGeom::filterMatchesPnP (vector<Point3f> points3d, vector<KeyPoint> keypoints,
                                     vector<DMatch> &matches) {
    Mat P;
    if ( matches.size() < 4 ) {
        P = Mat::eye (3, 3, CV_32F);
        return P;
    }

    vector<Point3f> matching_points3d;
    vector<Point2f> matching_points2d;
    match2points (points3d, keypoints, matches,
                  matching_points3d, matching_points2d);

	Mat tvec = Mat::zeros(3, 1, CV_32F);
	Mat rvec = Mat::zeros(3, 1, CV_32F);
	vector<float> distortion;
    vector<int> inliers_index;
    solvePnPRansac (matching_points3d, matching_points2d, K_, distortion,
                    rvec, tvec,
                    false, 400, 8.0, matching_points3d.size()*0.8, inliers_index, CV_EPNP);
                    //false, 800, 2.0, matching_points3d.size()*0.8, inliers_index, CV_EPNP);
                    //false, 2000, 8.0, 20000, inliers_index, CV_EPNP);

    int m = inliers_index.size();
    vector<DMatch> inliers (m);
    for ( size_t i = 0; i < m; ++i ) {
        inliers[i] = matches[inliers_index[i]];
    }
    matches.swap(inliers);

    P = vecToMat (rvec, tvec);
    return P;
}

Mat PipelineGeom::computePnP (vector<Point2f> corners2d, Mat H) {
    vector<Point2f> projected_corners2d (corners2d.size());
    perspectiveTransform(corners2d, projected_corners2d, H);

    unsigned int w = corners2d[3].x;
    unsigned int h = corners2d[3].y;
    float maxSize = std::max (w, h);
    float unitW = w / maxSize;
    float unitH = h / maxSize;

    vector<Point3f> corners3d (4);
    corners3d[0] = Point3d (-unitW, -unitH, 0);
    corners3d[1] = Point3d (unitW, -unitH, 0);
    corners3d[2] = Point3d (-unitW, unitH, 0);
    corners3d[3] = Point3d (unitW,  unitH, 0);
	Mat tvec = Mat::zeros(3, 1, CV_32F);
	Mat rvec = Mat::zeros(3, 1, CV_32F);
	vector<float> distortion;
    vector<int> inliers_index;

    solvePnP (corners3d, projected_corners2d, K_, distortion, rvec, tvec);
    //cout << tvec << endl;
    //cout << rvec << endl;
    Mat P = Mat::eye(4, 4, CV_32F);
    P = vecToMat (rvec, tvec);
    return P;
}

Mat PipelineGeom::warpImage (Mat image, Mat H, Size size) {
    Mat warped_image;
    //cv::warpPerspective(image, warped_image, H, size, cv::WARP_INVERSE_MAP | cv::INTER_CUBIC);
    cv::warpPerspective(image, warped_image, H, size, WARP_INVERSE_MAP | INTER_LANCZOS4);
    return warped_image;
}
//void PipelineGeom::warpAndComputeHomography () {}

////////////////////////////////////////////////////////////////////////////////
//  PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
void PipelineGeom::match2points (vector<KeyPoint> kpts1, vector<KeyPoint> kpts2, vector<DMatch> matches,
                                 vector<Point2f> &pts1, vector<Point2f> &pts2) {
  for( int i = 0; i < matches.size(); i++ ) {
    pts1.push_back( kpts1[matches[i].queryIdx ].pt );
    pts2.push_back( kpts2[matches[i].trainIdx ].pt );
  }
}

void PipelineGeom::match2points (vector<Point3f> point3d, vector<KeyPoint> keypoints, vector<DMatch> matches,
                                 vector<Point3f> &matching_point3d, vector<Point2f> &matching_point2d) {
  for( int i = 0; i < matches.size(); i++ ) {
    matching_point3d.push_back( point3d[matches[i].queryIdx ] );
    matching_point2d.push_back( keypoints[matches[i].trainIdx ].pt );
  }
}

Mat PipelineGeom::vecToMat (Mat rvec, Mat tvec) {
    Mat R;
	Rodrigues(rvec, R);

	Mat P = (Mat_<double>(4, 4, CV_64F) <<	R.at<double>(0,0),	R.at<double>(0,1),	R.at<double>(0,2), tvec.at<double>(0),
                                 			R.at<double>(1,0),	R.at<double>(1,1),	R.at<double>(1,2), tvec.at<double>(1),
			                                R.at<double>(2,0),	R.at<double>(2,1),	R.at<double>(2,2), tvec.at<double>(2),
                            			    0                ,                  0,                  0,                  1);
    return P;
}
