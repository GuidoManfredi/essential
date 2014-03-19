#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

class ICP {
 public:
  ICP ();
/*
  std::vector<float> align (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cluster,
														 std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> model,
														 std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &final_cloud,
														 std::vector<Eigen::Matrix4f> &final_transform);
*/
	float align (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cluster,
							 pcl::PointCloud<pcl::PointXYZ>::ConstPtr model,
							 Eigen::Matrix4f &final_transform);
														 
	float align (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cluster,
							 pcl::PointCloud<pcl::PointXYZ>::ConstPtr model,
							 pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud,
							 Eigen::Matrix4f &final_transform);							 
 private:
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> model_;
};
