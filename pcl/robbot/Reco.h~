#ifndef DEF_ROBBOT_RECO
#define DEF_ROBBOT_RECO

#include <boost/math/distributions/chi_squared.hpp>

#include <Eigen/Core>

#include "string_tools.h"
#include "PlanDetector.h"

class Reco
{
public:
	Reco (int dim);
	std::vector<std::string> reco (pcl::PointXYZ, float confidence);
	std::vector<std::string> reco (pcl::PointXYZ min, pcl::PointXYZ max, float confidence);
	
	void load_stats (std::string stats);
	void save_results (std::string pose_name);
	
	float mahalanobis_distance (Eigen::Matrix3f cov, Eigen::Vector4f norm_mean, pcl::PointXYZ pt);
	float max_distance (float percent);
	
protected:
	int _dim;

	std::vector<float> _vec_distance;	
	std::vector<float> _vec_distance_min;
	std::vector<float> _vec_distance_max;

	std::vector<Eigen::Matrix3f> _vec_cov;
	std::vector<Eigen::Vector4f> _vec_mean;
	std::vector<std::string> _vec_name;
};

#endif

