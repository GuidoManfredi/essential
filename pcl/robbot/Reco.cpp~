#include "Reco.h"

using namespace std;

Reco::Reco (int dim):
_dim (dim)
{}

vector<string> Reco::reco (pcl::PointXYZ desc, float confidence)
{
	vector<string> vec_candidats;
	_vec_distance.resize (_vec_cov.size ());

	for (size_t i=0; i<_vec_cov.size (); ++i)
	{
		_vec_distance[i] = mahalanobis_distance ( _vec_cov[i], _vec_mean[i], desc);
		if ( _vec_distance[i] <= max_distance (confidence))
			vec_candidats.push_back (_vec_name[i]);
	}
	
	return vec_candidats;
}

vector<string> Reco::reco (pcl::PointXYZ min, pcl::PointXYZ max, float confidence)
{
	vector<string> vec_candidats;
	_vec_distance_min.resize (_vec_cov.size ());
	_vec_distance_max.resize (_vec_cov.size ());

	for (size_t i=0; i<_vec_cov.size (); i+=2)
	{
		_vec_distance_min[i] = mahalanobis_distance ( _vec_cov[i], _vec_mean[i], min);
		_vec_distance_max[i] = mahalanobis_distance ( _vec_cov[i+1], _vec_mean[i+1], max);

		if ( _vec_distance_min[i] <= max_distance (confidence) && _vec_distance_max[i] <= max_distance (confidence))
			vec_candidats.push_back (_vec_name[i/2.0]);
	}
	
	return vec_candidats;
}

void Reco::load_stats (std::string stats)
{
	ifstream file (stats.c_str ());
	if (!file.is_open ())
		cout << "Couldn't open file " << stats << endl;
	
  std::string line;
  while (getline (file, line))
  {
	  std::vector<std::string> splitted;
		boost::split (splitted, line, boost::algorithm::is_any_of(" "));
		
		_vec_name.push_back (splitted[0]);
		if ( splitted.size() >= 4 )
		{
			Eigen::Vector4f mean;
			Eigen::Matrix3f cov;
			for (int i=0; i<3; ++i)
				mean[i] = atof (splitted[i + 1].c_str ());
			for (int i=0; i< 3; ++i)
				for (int j=0; j<3; ++j)
					cov (i,j) = atof (splitted[i*3+j + 4].c_str ()); // offset of 4

			_vec_mean.push_back (mean);
			_vec_cov.push_back (cov);					
		}
		if ( splitted.size() == 25 ) // color
		{
			Eigen::Vector4f mean;
			Eigen::Matrix3f cov;
			for (int i=0; i<3; ++i)
				mean[i] = atof (splitted[i + 13].c_str ());
			for (int i=0; i< 3; ++i)
				for (int j=0; j<3; ++j)
					cov (i,j) = atof (splitted[i*3+j + 16].c_str ()); // offset of 4

			_vec_mean.push_back (mean);
			_vec_cov.push_back (cov);							
		}
	}
}

void Reco::save_results (string pose_name)
{
  ofstream file;
  string path = "results/" + pose_name + "_reco_distances.csv";
  file.open (path.c_str(), std::ios_base::app);

	if (_vec_distance.size() != 0) {
	  for (size_t i = 0; i< _vec_distance.size(); ++i)
		  file << _vec_distance[i] << endl;
	}
	else if (_vec_distance_min.size() != 0 && _vec_distance_max.size() != 0) {
	  for (size_t i = 0; i< _vec_distance.size(); ++i)
		  file << _vec_distance_min[i] << " " << _vec_distance_min[i] << endl;
	}

  file.close();
}

// Returns squared mahalanobis distance
float Reco::mahalanobis_distance (Eigen::Matrix3f cov, Eigen::Vector4f mean, pcl::PointXYZ pt)
{
	Eigen::Vector3f diff (pt.x - mean[0], pt.y - mean[1], pt.z - mean[2]);
	return diff.dot(cov.inverse() * diff);
}

float Reco::max_distance (float percent)
{
	boost::math::chi_squared mydist(_dim); // Dimension N
	return boost::math::quantile(mydist, percent); 	// Area under curve
}
