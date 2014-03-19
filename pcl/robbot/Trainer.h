#ifndef DEF_ROBBOT_TRAINER
#define DEF_ROBBOT_TRAINER

#include <boost/filesystem.hpp>
#include <boost/lambda/bind.hpp>

#include <pcl/io/openni_grabber.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>

#include "string_tools.h"
#include "PlanDetector.h"

class Trainer
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

	Trainer (std::string name, std::string results_filename);

protected:
	std::string _name;
	std::string _results_filename;

	std::vector<Eigen::Matrix3f> _vec_instances_cov;
	std::vector<Eigen::Vector4f> _vec_instances_mean;
	std::vector<std::string> _vec_instances_name;

	std::vector<Eigen::Matrix3f> _vec_classes_cov;
	std::vector<Eigen::Vector4f> _vec_classes_mean;
	std::vector<std::string> _vec_classes_name;
	
  pcl::PointCloud<pcl::PointXYZ>::Ptr _class_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr _instance_cloud;
};

#endif

