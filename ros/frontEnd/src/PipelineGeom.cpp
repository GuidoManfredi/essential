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
	cout << K_ << endl;
	
	pose_estimator = new PoseEstimator(K);
	
	required_inliers_ = required_inliers;
}

Mat PipelineGeom::computePoseOpenCV (vector<Point3f> p3d, vector<Point2f> p2d,
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
	solvePnPRansac(p3d, p2d, K_, distCoeffs, rvec, tvec, false, 1000, 5.0, p3d.size()*0.8, inliers, CV_EPNP);
	
	size_t n = inliers.size();
	vector<Point3f> refined_p3d (n);
	vector<Point2f> refined_p2d (n);
	for (size_t i=0; i<n; ++i) {
		refined_p3d[i] = p3d[inliers[i]];
		refined_p2d[i] = p2d[inliers[i]];
	}
	//solvePnP(refined_p3d, refined_p2d, K_, distCoeffs, rvec, tvec, false, CV_EPNP); // useless, already done during ransac.
	//solvePnP(refined_p3d, refined_p2d, K_, distCoeffs, rvec, tvec, true, CV_EPNP);
	
	Mat R;
	Rodrigues(rvec, R);
	
	//R = R.t();  // rotation of inverse
    //tvec = -R * tvec; // translation of inverse
	
	Mat P = (Mat_<double>(4, 4, CV_64F) <<	R.at<double>(0,0),	R.at<double>(0,1),	R.at<double>(0,2), tvec.at<double>(0),
                                 			R.at<double>(1,0),	R.at<double>(1,1),	R.at<double>(1,2), tvec.at<double>(1),
			                                R.at<double>(2,0),	R.at<double>(2,1),	R.at<double>(2,2), tvec.at<double>(2),
                            			    0                ,                  0,                  0,                  1);
	
	return P;
}

Mat PipelineGeom::computePoseRPnP (vector<Point3f> p3d, vector<Point2f> p2d,
								   vector<int> &inliers) {
	assert (p3d.size() != 0 && "Error: computePose: No 3D points");
	assert (p2d.size() != 0 && "Error: computePose: No 2D points");
	
	vector<int> p3d_idx;
	vector<int> p2d_idx;
	for ( size_t i = 0; i < p3d.size(); ++i ) {
		p3d_idx.push_back (i);
		p2d_idx.push_back (i);
	}
	
	float max_reprojection_error = 3.0;
    pose_estimator->compute_pnp_ransac (p3d, p2d, p3d_idx, p2d_idx, max_reprojection_error, 1000);
    inliers = pose_estimator->get_inliers (p3d, p2d, p3d_idx, p2d_idx, max_reprojection_error);
    Mat P = Mat(pose_estimator->getP());
	
	return P;
}

double PipelineGeom::meanReprojectionError (std::vector<cv::Point3f> p3d, std::vector<cv::Point2f> p2d,
                                        Mat R, Mat t) {
  double error_sum = 0.0;
  for ( size_t i = 0; i < p3d.size(); i++ ) {
    double error = 0;
    double Xc = R.at<double>(0,0) * p3d[i].x
                + R.at<double>(0,1) * p3d[i].y
                + R.at<double>(0,2) * p3d[i].z
                + t.at<double>(0);
    double Yc = R.at<double>(1,0) * p3d[i].x 
                + R.at<double>(1,1) * p3d[i].y
                + R.at<double>(1,2) * p3d[i].z
                + t.at<double>(1);
    double Zc = R.at<double>(2,0) * p3d[i].x 
                + R.at<double>(2, 1) * p3d[i].y
                + R.at<double>(2, 2) * p3d[i].z
                + t.at<double>(2);
    //cout << Xc/Zc << " " << Yc/Zc << " " << p2d[i].x << " " << p2d[i].y << endl;
    double inv_Zc = 1.0 / Zc;
    double ue = Xc * inv_Zc * K_.at<double>(0,0) + K_.at<double>(0,2);
    double ve = Yc * inv_Zc * K_.at<double>(1,1) + K_.at<double>(1,2);
    //double u = (p2d[i].x- K_.at<double>(0,2)) / K_.at<double>(0,0);
    //double v = (p2d[i].y- K_.at<double>(1,2)) / K_.at<double>(1,1);
    double u = p2d[i].x;
    double v = p2d[i].y;
    //cout << u << " " << v << " " << ue << " " << ve << endl;
    error = sqrt( (u - ue) * (u - ue) + (v - ve) * (v - ve) );
    //cout << error << endl;
    error_sum += error;
  }

  return error_sum / p3d.size();
}

double PipelineGeom::meanReprojectionError2 (std::vector<cv::Point3f> p3d, std::vector<cv::Point2f> p2d,
                                                Mat R, Mat t) {
    double error_sum = 0.0;
    Mat rvec = Mat::zeros(3, 1, CV_32F);
    Mat tvec = Mat::zeros(3, 1, CV_32F);
    vector<float> d;
    vector<Point2f> proj_p2d;
    projectPoints (p3d, rvec, tvec, K_, d, proj_p2d);
    for(size_t i = 0; i < p3d.size(); i++) {
        double error = norm (p2d[i] - proj_p2d[i]);
        error_sum += error;
    }

    return error_sum / p3d.size();
}
// Apparently this is RMS reporjection error ( cf. http://opencv-users.1802565.n2.nabble.com/What-is-the-reprojection-error-in-cvCalibrateCamera2-supposed-to-represent-td5816484.html )
double PipelineGeom::meanReprojectionError3 (std::vector<cv::Point3f> p3d, std::vector<cv::Point2f> p2d,
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




