#include "BoundingBoxer.h"

using namespace pcl;
using namespace std;

void BoundingBoxer::bounding_box_pca (Cloud::ConstPtr cloud,
																			 Eigen::Quaternionf &quaternion,
																			 Eigen::Vector3f &translation,
				 															 double &width, double &height, double &depth) {
	cout << "Computing bounding box..." << endl;
	
	pcl::PCA<PointType> pca;
	pca.setInputCloud(cloud);
	Eigen::Vector4f center = pca.getMean();
	translation(0) = center(0);
	translation(1) = center(1);
	translation(2) = center(2);
	Eigen::Matrix3f R = pca.getEigenVectors();
	R.col(2) = R.col(0).cross(R.col(1));
	/******* GOD KNOWS WHY *****/
	Eigen::Vector3f tmp_vec;
	tmp_vec = R.col(0);
	R.col(0) = -R.col(1);
	R.col(1) = -R.col(2);
	R.col(2) = tmp_vec;
	/**************************/
	cout << R << endl;
	quaternion = Eigen::Quaternionf (R);
	
	Eigen::Vector3f rpy = R.eulerAngles(0, 1, 2);
	Eigen::Affine3f t;
	pcl::getTransformation (center(0), center(1), center(2),
													rpy(0),	rpy(1), rpy(2), t);
	
	Cloud::Ptr tmp (new Cloud);
	pcl::transformPointCloud (*cloud, *tmp, t.inverse());
	
	Eigen::Vector4f min, max;
	pcl::getMinMax3D (*tmp, min, max);
	cout << "Min: " << endl << min << endl << "Max: " << endl << max << endl;
	width = fabs(max(0) - min(0));
	height = fabs(max(1) - min(1));
	depth = fabs(max(2) - min(2));
/*	
	cout << "W: " << width << " H: " << height << " D: " << depth << endl;
	cout << "Translation: " << translation.x() << " " << translation.y() << " " << translation.z() << endl;
	cout << "Rotation: " << quaternion.w() << " " << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << endl;
*/
}

void BoundingBoxer::bounding_box_mvbb (Cloud::ConstPtr cloud,
																			 Eigen::Quaternionf &quaternion,
																			 Eigen::Vector3f &translation,
				 															 double &width, double &height, double &depth) {
	cout << "Computing bounding box..." << endl;
	int n = cloud->size();
	gdiam_real* points = (gdiam_real*) malloc (3*n*sizeof(gdiam_real));
	pcl2gdiam (cloud, points);
	
	GPointPair pair;	
  pair = gdiam_approx_diam_pair( points, n, 0.0 );

  gdiam_point  * pnt_arr;
  pnt_arr = gdiam_convert( (gdiam_real *)points, n );

  gdiam_bbox bb;
  bb = gdiam_approx_mvbb_grid_sample( pnt_arr, n, 1, 400 );
  
	PointType min (bb.low_1, bb.low_2, bb.low_3);
	PointType max (bb.high_1, bb.high_2, bb.high_3);
	//cout << "Min: " << min << endl << "Max: " << max << endl;
	width = fabs(max.x-min.x);
	height = fabs(max.y-min.y);
	depth = fabs(max.z-min.z);

	gdiam_point_t centre;
	bb.get_centre (centre);
	translation = Eigen::Vector3f (centre[0], centre[1], centre[2]);

	Eigen::Matrix3f R;
	R(0, 0) = bb.dir_1[0];		R(0, 1) = bb.dir_2[0];		R(0, 2) = bb.dir_3[0];
	R(1, 0) = bb.dir_1[1];		R(1, 1) = bb.dir_2[1];		R(1, 2) = bb.dir_3[1];
	R(2, 0) = bb.dir_1[2];		R(2, 1) = bb.dir_2[2];		R(2, 2) = bb.dir_3[2];
	quaternion = Eigen::Quaternionf (R);
/*
	cout << R << endl;

	cout << "W: " << width << " H: " << height << " D: " << depth << endl;
	cout << "Translation: " << translation.x() << " " << translation.y() << " " << translation.z() << endl;
	cout << "Rotation: " << quaternion.w() << " " << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << endl;
*/
}

void BoundingBoxer::pcl2gdiam (Cloud::ConstPtr cloud, gdiam_real* points)
{
	for (size_t i=0; i<cloud->size (); ++i)
	{
		points[ 3 * i ] = cloud->points[i].x;
		points[ 3 * i + 1 ] = cloud->points[i].y;
		points[ 3 * i + 2 ] = cloud->points[i].z;
	}
}
