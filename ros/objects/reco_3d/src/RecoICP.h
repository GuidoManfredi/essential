#include <iostream>
#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>

#include "ICP.h"

class RecoICP {
 public:
	RecoICP ();
	std::vector<std::pair<std::string, float> >
	recognize (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
						 Eigen::Matrix4f &final_transform,
						 std::vector<std::string> candidates = std::vector<std::string>());
	std::vector< std::pair<std::string, float> >
	recognize (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
						 std::vector<Eigen::Matrix4f> &final_transform,
						 std::vector<std::string> candidates = std::vector<std::string>());
	int loadModels (std::string pcd_dir_path);
 private:
  int getMinFitnessIndex (std::vector<std::pair<std::string,float> > fitness);
 
  ICP icp_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> model_;
  std::vector<std::string> model_name_;
};
