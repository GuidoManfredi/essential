#ifndef DEF_RECO_3D_RECO
#define DEF_RECO_3D_RECO

#include <fstream>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/math/distributions/chi_squared.hpp>
#include <Eigen/Core>
#include "pcl/point_types.h"

class Reco
{
 public:
	Reco ();
	std::vector<std::string> recognize (float w, float h, float d, float confidence, 
																 std::vector<std::string> candidates = std::vector<std::string>());
	void load_training_file (std::string stats_filepath);	

 protected:
	std::vector<std::string> recognize (pcl::PointXYZ descriptor, float confidence,
																 			std::vector<std::string> candidates = std::vector<std::string>());
	float mahalanobis_distance (Eigen::Matrix3f cov, Eigen::Vector3f norm_mean, pcl::PointXYZ pt);

	std::vector<Eigen::Matrix3f> _vec_cov;
	std::vector<Eigen::Vector3f> _vec_mean;
	std::vector<std::string> _vec_names;
};

#endif

