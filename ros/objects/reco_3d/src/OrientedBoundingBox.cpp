#include "OrientedBoundingBox.h"

OrientedBoundingBox::OrientedBoundingBox (){}

void OrientedBoundingBox::compute_obb_pca_hull (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
																								 Eigen::Quaternionf &quaternion,
																								 Eigen::Vector3f &translation,
										 														 Eigen::Vector3f &dims) {
}

void OrientedBoundingBox::compute_obb_pca (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
																					 Eigen::Quaternionf &quaternion,
																					 Eigen::Vector3f &translation,
							 														 Eigen::Vector3f &dims) {
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(cloud);
	// Compute Mean
	Eigen::Vector4f center = pca.getMean();
	translation << center(0), center(1), center(2);
	// Compute eigen vectors from covariance matrix
	Eigen::Matrix3f R = pca.getEigenVectors();
	R.col(0).normalize();
	R.col(1).normalize();
	R.col(2) = R.col(0).cross(R.col(1)).normalized();
	Eigen::Matrix3f rot;
	// Order the eigen vectors according to the determinant value 
	// TODO test if this is necessary, have the feeling 
	// reordering is already done in PCA class.
	/*
	if (R.determinant() >= 0) {
	  rot.col(0) << R.col(0);
	  rot.col(1) << R.col(1);
  	rot.col(2) << (R.col(0).cross(R.col(1))).normalized();
	}
	else {
	  rot.col(0) << (R.col(0).cross(R.col(1))).normalized();
	  rot.col(1) << R.col(0);
  	rot.col(2) << R.col(1);
	}
	*/
  rot.col(0) << R.col(0);
  rot.col(1) << R.col(1);
	rot.col(2) << (R.col(0).cross(R.col(1))).normalized();
	quaternion = Eigen::Quaternionf(rot);

	// Transform all points into the obb frame to get obb dimensions and correct center
	Eigen::Matrix4f T, T_inv;
  T.col(0) << rot.col(0), 0;
	T.col(1) << rot.col(1), 0;
	T.col(2) << rot.col(2), 0;
	T.col(3) << translation, 1;
	T_inv = T.inverse();
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud (*cloud, *tmp, T_inv);
	Eigen::Vector4f min, max;
	pcl::getMinMax3D (*tmp, min, max);
	dims[0] = fabs(max[0] - min[0]);
	dims[1] = fabs(max[1] - min[1]);
	dims[2] = fabs(max[2] - min[2]);
	
	Eigen::Vector4f t;
	t[0] = (max[0] + min[0]) / 2;
	t[1] = (max[1] + min[1]) / 2;
	t[2] = (max[2] + min[2]) / 2;
	t[3] = (max[3] + min[3]) / 2;
	center = T*t;
	translation << center(0), center(1), center(2);
	
}

