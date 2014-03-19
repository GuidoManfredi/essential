#include "tools.h"

using namespace std;

cv::Mat draw_points_labels (unsigned width, unsigned height, std::vector<cv::Point> pts, cv::Mat labels) {
	cv::Mat res = cv::Mat::zeros (height, width, CV_8UC3);
	std::vector<cv::Vec3b> colors;
	colors.push_back (cv::Vec3b(255, 0, 0));
	colors.push_back (cv::Vec3b(0, 255, 0));
	colors.push_back (cv::Vec3b(0, 0, 255));
	colors.push_back (cv::Vec3b(255, 255, 255));
	colors.push_back (cv::Vec3b(0, 255, 255));
	for (size_t i = 0; i < pts.size(); ++i) {
		//cout << "Color = " << colors[labels.at<unsigned char>(i)] << endl;
		//cout << int(labels.at<unsigned char>(i)) << endl;
		res.at<cv::Vec3b>(pts[i].y, pts[i].x) = colors[labels.at<unsigned char>(i)];
	}
	return res;
}

cv::Mat points_to_mat (int width, int height, std::vector<cv::Point> points) {
	cv::Mat res = cv::Mat::zeros (height, width, CV_8UC1);
	for (size_t n=0; n<points.size(); ++n)
		res.at<unsigned char>(points[n].y, points[n].x) = 255;	
	return res;
}

octomap::point3d point_to_voxel_coord (octomap::point3d point,
																											  double resolution,
																											  octomap::point3d min) {
	unsigned int x = (point.x() - min.x()) / resolution;
	unsigned int y = (point.y() - min.y()) / resolution;
	unsigned int z = (point.z() - min.z()) / resolution;
	return octomap::point3d(x, y, z);
}

octomap::point3d voxel_coord_to_point (octomap::point3d coord,
																												double resolution,
																												octomap::point3d min) {
	double x = coord.x() * resolution + min.x() ;
	double y = coord.y() * resolution + min.y() ;
	double z = coord.z() * resolution + min.z() ;
	return octomap::point3d(x, y, z);
}

void bin_to_z (unsigned int bin, double resolution,
																double &min_z, double &max_z) {
	min_z = bin * resolution;
	max_z = (bin+1) * resolution;
}
