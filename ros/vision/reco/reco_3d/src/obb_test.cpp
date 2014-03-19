#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "OrientedBoundingBox.h"

pcl::visualization::PCLVisualizer cloud_viewer("PCL Display");

int main (int argc, char const* argv[])
{
	if (argc != 2) {
		cout << "Usage : obb_test filename.pcd" << endl;
		return 1;
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 	if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return 1;
  }

	cloud_viewer.addPointCloud (cloud, "single_cloud");

	OrientedBoundingBox obb;
	Eigen::Quaternionf q;
	Eigen::Vector3f t, dims;
	obb.compute_obb_pca (cloud, q, t, dims);
  cloud_viewer.addCube(t, q, dims.x(), dims.y(), dims.z());
  cout << dims.x() << " " << dims.y() << " " << dims.z() << endl;
  
  while (!cloud_viewer.wasStopped()) {
    cloud_viewer.spinOnce(1);
	}
  
	return 0;
}
