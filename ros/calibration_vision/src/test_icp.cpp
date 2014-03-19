#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int
main (int argc, char** argv)
{
	assert (argc == 3 && "Usage : test_icp maximum_correspondence_distance euclidean_fitness_epsilon");

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the CloudIn data
  cloud_in->width    = 10;
  cloud_in->height   = 1;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {
    cloud_in->points[i].x = 0;
    cloud_in->points[i].y = 0;
    cloud_in->points[i].z = 1 * i;
  }

  *cloud_out = *cloud_in;
  std::cout << "size:" << cloud_out->points.size() << std::endl;
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
    cloud_out->points[i].z = cloud_in->points[i].z + 0.7f;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaxCorrespondenceDistance (atof(argv[1]));
	icp.setTransformationEpsilon (1e-8);
	icp.setEuclideanFitnessEpsilon (atof(argv[2]));
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

 return (0);
}
