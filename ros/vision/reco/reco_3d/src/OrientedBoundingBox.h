#ifndef __NAME_H__
#define __NAME_H__

#include <pcl/common/pca.h>
#include <pcl/point_types.h>
#include <iostream>

class OrientedBoundingBox
{
	public:
		OrientedBoundingBox ();
		void compute_obb_pca_hull (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
															 Eigen::Quaternionf &quaternion,
															 Eigen::Vector3f &translation,
															 Eigen::Vector3f &dims);
		void compute_obb_pca (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
													 Eigen::Quaternionf &quaternion,
													 Eigen::Vector3f &translation,
													 Eigen::Vector3f &dims);
	private:
		/* data */
};

#endif /* __NAME_H__ */

