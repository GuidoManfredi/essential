#include "PipelineGeom.h"

using namespace cv;
using namespace std;

PipelineGeom::PipelineGeom (std::string calibration_file,
                            unsigned int required_inliers) {
	Mat K(3,3,CV_32F);
    Mat d(5,1,CV_32F);
    FileStorage r_fs;
    r_fs.open (calibration_file, cv::FileStorage::READ);
    r_fs["camera_matrix"]>>K;
    r_fs["distortion_coefficients"]>>d;
    r_fs.release ();
	K_ = K;
	d_ = d;

	required_inliers_ = required_inliers;
}

Mat PipelineGeom::computePose (vector<Point3f> p3d, vector<Point2f> p2d,
                               vector<int> &inliers) {
    inliers.clear();
	//assert (p3d.size() >= 4 && "Error: computePose: No 3D points");
	//assert (p2d.size() >= 4 && "Error: computePose: No 2D points");
	if (p3d.size() < 4 || p2d.size() < 4)
	    return Mat::eye(4, 4, CV_64F);

	Mat tvec = Mat::zeros(3, 1, CV_32F);
	Mat rvec = Mat::zeros(3, 1, CV_32F);
	vector<float> distCoeffs; // empty, we don't use it as images are already rectified
	// Stop if 80 percent of points are inliers
	solvePnPRansac(p3d, p2d, K_, distCoeffs, rvec, tvec, false, 300, 5.0, p3d.size()*0.8, inliers, CV_EPNP);

	size_t n = inliers.size();
	vector<Point3f> refined_p3d (n);
	vector<Point2f> refined_p2d (n);
	for (size_t i=0; i<n; ++i) {
		refined_p3d[i] = p3d[inliers[i]];
		refined_p2d[i] = p2d[inliers[i]];
	}

	Mat R;
	Rodrigues(rvec, R);
	Mat P = (Mat_<double>(4, 4, CV_64F) <<	R.at<double>(0,0),	R.at<double>(0,1),	R.at<double>(0,2), tvec.at<double>(0),
                                 			R.at<double>(1,0),	R.at<double>(1,1),	R.at<double>(1,2), tvec.at<double>(1),
			                                R.at<double>(2,0),	R.at<double>(2,1),	R.at<double>(2,2), tvec.at<double>(2),
                            			    0                ,                  0,                  0,                  1);

	return P;
}

// Apparently this is RMS reporjection error ( cf. http://opencv-users.1802565.n2.nabble.com/What-is-the-reprojection-error-in-cvCalibrateCamera2-supposed-to-represent-td5816484.html )
double PipelineGeom::meanReprojectionError (std::vector<cv::Point3f> p3d, std::vector<cv::Point2f> p2d,
                                                Mat R, Mat t) {
    Mat rvec = Mat::zeros(3, 1, CV_32F);
    Mat tvec = Mat::zeros(3, 1, CV_32F);
    vector<float> d;
    vector<Point2f> proj_p2d;
    projectPoints (p3d, rvec, tvec, K_, d, proj_p2d);

    double error = norm (Mat(p2d), Mat(proj_p2d), CV_L2);
    int num_points = p3d.size();
    return sqrt(error*error/num_points);
}




