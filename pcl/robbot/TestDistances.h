#ifndef DEF_ROBBOT_TESTDISTANCES
#define DEF_ROBBOT_TESTDISTANCES

#include <boost/math/distributions/chi_squared.hpp>

#include <Eigen/Core>

#include "string_tools.h"
#include "PlanDetector.h"
#include "Reco.h"

class TestDistances
{
 public:
	TestDistances ();
	~TestDistances ();
	// Be carefull, when processing color stats, the returned vector size is double
	// the size of the data. When sorting for best/worst, sort first half independently
	// from second half (min then max)
	void retrieve_variances (std::string stats_filepath,
														float percent_example_start,
														float percent_example_end,	
														std::vector<float> &variances_x,
														std::vector<float> &variances_y,
														std::vector<float> &variances_z);
									
	void compute_candidates ( std::string dataset_path,
														std::string result_file,
														std::string stats_filepath,
														float percent_example_start,
														float percent_example_end,
														float confidence,
														std::vector< std::vector<std::string> > &candidates);
																	
	void compute_positive_negative_rates (std::string dataset_path,
																				float &true_positives, float &true_negatives,
																				float &false_positives, float &false_negatives);

	// Test set, name of results files
	void process_root (std::string full_path, std::string results_type, float start, float end);
	
	void get_candidates (std::vector<pcl::PointXYZ> boxes_dimensions, float confidence);
	
	std::vector<pcl::PointXYZ> read_boxes_dimensions (std::string results_filepath,
																										float percent_example_start,
																										float percent_example_end);
	
	float get_num_lines (std::string filename);
	
 protected:
	Reco* _recognition;
	// A vector of candidates for each test pose
	std::vector<std::string> _all_names;
	std::vector< std::vector<std::string> > _all_candidates;

/*
	std::vector<float> _true_positive;
	std::vector<float> _false_positive;	
	std::vector<float> _true_negative;	
	std::vector<float> _false_negative;		
*/
	
};

#endif

