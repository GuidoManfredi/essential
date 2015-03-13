#include "PipelineGeom.h"

#include "opencv2/core/core_c.h" // fore cv::reduce

using namespace std;
using namespace cv;

PipelineGeom::PipelineGeom () {}

void PipelineGeom::mySolvePnP (Mat p3d, Mat p2d, Mat K,
                               Mat &Rov, Mat &tov,
                               Mat &p3d_inliers, Mat &p2d_inliers) {
    Mat rvec = Mat::zeros (1, 3, CV_32F);
    Rov = Mat::eye (3, 3, CV_32F);
    tov = Mat::zeros (1, 3, CV_32F);
//    cout << p3d << endl;
//    cout << p2d << endl;
    vector<int> inliers_idx;
    solvePnPRansac (p3d, p2d, K, vector<float>(), rvec, tov,
                    false, 200, 2.0, p3d.rows*0.95, inliers_idx); // 99% inliers
//                    false, 500, 8.0, p3d.rows*0.80, inliers_idx); // 99% inliers
    p3d_inliers = p3d;
    p2d_inliers = p2d;
    /*
    solvePnPRansac (p3d, p2d, K, vector<float>(), rvec, tov,
                    false, 500, 2.0, p3d.rows*0.99, inliers_idx, CV_EPNP); // 80% inliers
    // Reffine with LM
    if ( inliers_idx.size() > 4 ) {
        pointsFromIndex (p3d, p2d, inliers_idx, p3d_inliers, p2d_inliers);
        solvePnP (p3d_inliers, p2d_inliers, K, vector<float>(), rvec, tov, true);
    } else {
        p3d_inliers = p3d;
        p2d_inliers = p2d;
    }
    */
    Rodrigues (rvec, Rov);

    Rov.convertTo (Rov, CV_32F);
    tov.convertTo (tov, CV_32F);
}

void PipelineGeom::pointsFromIndex (Mat p3d, Mat p2d, vector<int> inliers,
                                    Mat &p3d_inliers, Mat &p2d_inliers) {
    p3d_inliers = Mat::zeros (inliers.size(), 3, CV_32F);
    p2d_inliers = Mat::zeros (inliers.size(), 2, CV_32F);
    for ( size_t n = 0; n < inliers.size(); ++n ) {
        int idx = inliers[n];
        p3d_inliers.at<Point3f>(n) = p3d.at<Point3f>(idx);
        p2d_inliers.at<Point2f>(n) = p2d.at<Point2f>(idx);
    }
}
