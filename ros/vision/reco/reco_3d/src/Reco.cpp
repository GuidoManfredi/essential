#include "Reco.h"

using namespace std;

Reco::Reco ()
{}

vector<string> Reco::recognize (float w, float h, float d, float max_distance,
													 std::vector<string> candidates)
{
	// w, h, d are in meters, convert to millimeters
	vector<float> v;
	v.push_back(w*1000);  v.push_back(h*1000);  v.push_back(d*1000);
	// order in descending order
	sort(v.begin(), v.end(), std::greater<int>());
	return recognize (pcl::PointXYZ(v[0], v[1], v[2]), max_distance, candidates);
}

void Reco::load_training_file (std::string stats_filepath)
{
	ifstream file (stats_filepath.c_str ());
	if (!file.is_open ())
		cout << "Couldn't open file " << stats_filepath << endl;
	
  std::string line;
  while (getline (file, line))
  {
	  vector<string> splitted;
		boost::algorithm::split (splitted, line, boost::algorithm::is_any_of(" "));
		
		_vec_names.push_back (splitted[0]);
		if ( splitted.size() == 13 )	{
			Eigen::Vector3f mean;
			Eigen::Matrix3f cov;
			for (int i=0; i<3; ++i)
				mean[i] = atof (splitted[i + 1].c_str ());
			for (int i=0; i< 3; ++i)
				for (int j=0; j<3; ++j)
					cov (i,j) = atof (splitted[i*3+j + 4].c_str ()); // offset of 1+3
			//cout << mean << endl;
			//cout << cov << endl;
			_vec_mean.push_back (mean);
			_vec_cov.push_back (cov);
		}	else {
			cout << "Error : load_stats : illegal number of values, found " 
					 << splitted.size() << endl;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
//	Private methods
////////////////////////////////////////////////////////////////////////////////

vector<string> Reco::recognize (pcl::PointXYZ descriptor, float max_distance,
													 			vector<string> candidates)
{
	vector<string> vec_candidates;
	map<float, string> map_distance_candidates;

	for (size_t i=0; i<_vec_cov.size (); ++i) {
		// Name must be among candidates
		if (candidates.empty() ||
		    find(candidates.begin(), candidates.end(), _vec_names[i]) != candidates.end()) {
  		
			float distance = mahalanobis_distance ( _vec_cov[i], _vec_mean[i], descriptor);
			//if (distance <= max_distance)
			pair<float, string> distance_candidate (distance, _vec_names[i]);
			map_distance_candidates.insert (map_distance_candidates.begin(), distance_candidate);
		}
	}
	// Map is ordered by decreasing distance
	for (std::map<float,string>::iterator it=map_distance_candidates.begin(); it!=map_distance_candidates.end(); ++it) {
		//cout << descriptor << endl << it->first << endl;
		vec_candidates.push_back (it->second);
	}
	
	return vec_candidates;
}

// Returns squared mahalanobis distance
float Reco::mahalanobis_distance (Eigen::Matrix3f cov, Eigen::Vector3f mean, pcl::PointXYZ pt)
{
	Eigen::Vector3f diff (pt.x - mean[0], pt.y - mean[1], pt.z - mean[2]);
	//return diff.dot(cov.inverse() * diff);
	return sqrt(diff.dot(cov.inverse() * diff));
}
