#ifndef DEF_ROBBOT_TRAINERCOLOR
#define DEF_ROBBOT_TRAINERCOLOR

#include <boost/filesystem.hpp>
#include <boost/lambda/bind.hpp>

#include <pcl/io/openni_grabber.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>

#include "string_tools.h"
#include "PlanDetector.h"

class TrainerColor
{
public:
	void process_root (std::string path, float start, float end);
	int process_class (std::string path, float start, float end);
	int process_instance (std::string path, float start, float end);
	
	void instance_stats (std::string filename);
	void class_stats (std::string filename);	
	void save_results ();
	
	float get_num_lines (std::string filename);
	float get_num_dirs (std::string dirname);

	TrainerColor (std::string name, std::string results_filename);

protected:
	std::string _name;
	std::string _results_filename;

	// Contains cov_min1/cov_max1/cov_min2/cov_max2/etc.
	std::vector<Eigen::Matrix3f> _vec_instances_cov;
	// same than for cov
	std::vector<Eigen::Vector4f> _vec_instances_mean;
	std::vector<std::string> _vec_instances_name;

	std::vector<Eigen::Matrix3f> _vec_classes_cov;
	std::vector<Eigen::Vector4f> _vec_classes_mean;
	std::vector<std::string> _vec_classes_name;
	
  pcl::PointCloud<pcl::PointXYZ>::Ptr _class_min_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr _class_max_cloud;
    
  pcl::PointCloud<pcl::PointXYZ>::Ptr _instance_min_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr _instance_max_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr _tmp_min_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr _tmp_max_cloud;  
};

#endif

