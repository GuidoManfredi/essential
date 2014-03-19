#include "TestDistances.h"

using namespace std;

int main (int argc, char** argv) {
	TestDistances test;

	// Test 2
	vector< vector< string > > candidates;
	string dataset_path = "../../dataset/rgbd-dataset/";
	string results_name = "alpha_xyz_res.txt";
	string stats_path = "xyz_instance_stats.txt";
	test.compute_candidates (dataset_path,
																results_name,
																stats_path,
																0.0,
																1.0,
																0.90,
																candidates);
												
	cout << candidates.size() << endl;
	for (size_t i=0; i <candidates.size(); ++i)
		cout << candidates[i].size() << endl;

	return 0;
}
