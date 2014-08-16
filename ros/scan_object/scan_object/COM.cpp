#include "COM.h"

using namespace std;
using namespace cv;

void COM::addFrame (cv::Mat image, cv::Mat depth, cv::Mat P) {
    // Acquire new image
    Mat gray;
    pipe2d_.getGray (image, gray);
    std::vector<cv::KeyPoint> raw_kpts, kpts;
    pipe2d_.detectFeatures (gray, raw_kpts);
    vector<Point3f> p3d;
    filterNaNKeyPoints (depth, raw_kpts, kpts, p3d); // remove NaN of depth from keypoints
    // Compute descriptors
    Mat descs;
    pipe2d_.describeFeatures (gray, kpts, descs);
    addDescriptors(descs);

    // Apply transform to 3D points
    vector<Point3f> transformed_p3d;
    transform (p3d, P, transformed_p3d);
    // Refine 3d pose
    refine (p3d, transformed_p3d);
    addPoints(transformed_p3d);
    // Remove duplicate points
    removeDuplicates();
}
///////////////////////////////////////////////////////////////////////////////
// PRIVATE METHODS
///////////////////////////////////////////////////////////////////////////////
void COM::transform (vector<Point3f> pts_in, Mat P,
                     vector<Point3f> &pts_out) {
    pts_out.resize(pts_in.size());

    Mat R, t;
    P2Rt(P, R, t);
    Mat out = R * Mat(pts_in) + t;
    out.copyTo(pts_out);
}

void COM::refine (vector<Point3f> pts1, vector<Point3f> pts2) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd1 (new pcl::PointCloud<pcl::PointXYZ>);
    vec2pcd (pts1, pcd1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd2 (new pcl::PointCloud<pcl::PointXYZ>);
    vec2pcd (pts2, pcd2);

    icp_.setInputCloud(pcd1);
    icp_.setInputTarget(pcd2);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp_.align(Final);
    std::cout << "has converged:" << icp_.hasConverged() << " score: " <<
    icp_.getFitnessScore() << std::endl;
    std::cout << icp_.getFinalTransformation() << std::endl;
}

void COM::addDescriptors(Mat descs) {
    vconcat(descs_, descs, descs_);
}

void COM::addPoints(vector<Point3f> p3d) {
    p3d_.insert(p3d_.end(), p3d.begin(), p3d.end());
}

void COM::removeDuplicates () {

}

void COM::filterNaNKeyPoints (Mat depth, vector<KeyPoint> kpts,
								vector<KeyPoint> &filtered_kpts,
								vector<Point3f> &filtered_p3d) {
	filtered_kpts.clear();
	filtered_p3d.clear();
	for (size_t i=0; i<kpts.size(); ++i) {
		Vec3f p3 = depth.at<Vec3f>(kpts[i].pt.y, kpts[i].pt.x); //* 1e3; // in meters, convert in milimeters
		if ( !isnan(p3[0]) && !isnan(p3[1]) && !isnan(p3[2]) ) {
			filtered_kpts.push_back (kpts[i]);
			filtered_p3d.push_back (p3); // looks like Vec3f ~= Point3f
		}
	}
}
