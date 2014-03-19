#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

// ./recalage_icp ../models/arm_segment_last.pcd ../scenes/arm_segment_last_scene1.pcd

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the CloudIn data
  pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *model);
  std::cout << "Model " << model->points.size () << "  points." << std::endl;
  pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *scene);
  std::cout << "Scene " << scene->points.size () << "  points." << std::endl;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaxCorrespondenceDistance (0.6);
  //icp.setUseReciprocalCorrespondences (true);
  icp.setMaximumIterations (500);
  icp.setInputCloud(model);
  icp.setInputTarget(scene);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "Has converged : " << icp.hasConverged() << std::endl
  					<< " score : " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

	pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  pcl::transformPointCloud (*model, transformed_cloud, icp.getFinalTransformation());
  pcl::io::savePCDFileBinary ("../recalage_icp_output.pcd", transformed_cloud);

 return (0);
}
