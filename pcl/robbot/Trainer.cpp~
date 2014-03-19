#include "Trainer.h"

using namespace std;
using namespace boost::filesystem;

typedef vector<path> Vec_path;

Trainer::Trainer (std::string name, std::string results_filename):
_name (name),
_results_filename (results_filename),
_class_cloud (new pcl::PointCloud<pcl::PointXYZ>),
_instance_cloud (new pcl::PointCloud<pcl::PointXYZ>)
{}

void Trainer::process_root (std::string dataset_path, float start, float end)
{
	if (is_directory (dataset_path))
  {
  	directory_iterator end_itr;
    for (directory_iterator i (dataset_path); i != end_itr; ++i)
    {
			if (is_directory (i->path().string()) )
			{
				process_class(i->path().string(), start, end);
			}
    }
  }
  
  cout << "Stats : " << _vec_classes_cov.size () <<  " " << _vec_instances_cov.size () << endl;
  save_results ();
}

int Trainer::process_class (std::string dir, float fStart, float fEnd)
{
	if (is_directory (dir))
  {
  	_class_cloud->clear ();
  
	  // sort, directory iteration is not ordered
    Vec_path v; // so we can sort them later
    copy(directory_iterator(dir), directory_iterator(), back_inserter(v));
    sort(v.begin(), v.end());
    
    // compute number of files
    int n = v.size ();
    int iStart = n*fStart;
    int iEnd = n*fEnd;
    
    for (int i = iStart; i < iEnd; ++i)
    {
    	process_instance (v[i].string(), fStart, fEnd);
    	*_class_cloud += *_instance_cloud;
    }
    
    class_stats ( path2name(dir) );
  }
}

int Trainer::process_instance (std::string instance_path, float start, float end)
{
	if (is_directory (instance_path))
  {
		// Clean instance cloud
		_instance_cloud->clear ();

		string results_path = instance_path + "/" + _results_filename;
		//cout << results_path << endl;
		float n = get_num_lines (results_path);
		int iStart = n*start;
		int iEnd = n*end;
	
		ifstream file (results_path.c_str());
		if (!file.is_open ())
		{
			cout << "Couldn't open instance file " << results_path.c_str() << ". Aborting" << endl;
			return -1;
		}
				
		std::string line;
		for (size_t i=iStart; i < iEnd; ++i)
		{
			getline (file, line);
			std::vector<std::string> splitted;
			boost::split (splitted, line, boost::algorithm::is_any_of(" "));

			if ( splitted.size () == 4 )
			{
				pcl::PointXYZ pt;
				vector<float> vals;
				vals.push_back(atof(splitted[1].c_str()));
				vals.push_back(atof(splitted[2].c_str()));
				vals.push_back(atof(splitted[3].c_str()));
				std::sort(vals.begin(), vals.end());
				// x is the biggest dim and z the smallest one
				pt.x = vals[2];
				pt.y = vals[1];
				pt.z = vals[0];
				// Degenerate cases have one dimension null, ignore them
				if ( pt.x != 0 && pt.y != 0 && pt.z != 0)
					_instance_cloud->push_back (pt);
			}			
		}
		file.close ();
	
		instance_stats (path2name(instance_path));
	}
}

void Trainer::instance_stats (std::string filename)
{
	//EIGEN_ALIGN16 Eigen::Matrix3f cov_mat;
	Eigen::Matrix3f cov_mat;
	Eigen::Vector4f centroid;
	pcl::computeMeanAndCovarianceMatrix (*_instance_cloud, cov_mat, centroid);
	
	//cout << filename << std::endl << cov_mat << std::endl << centroid << std::endl;				
	_vec_instances_cov.push_back (cov_mat);
	_vec_instances_mean.push_back (centroid);
	_vec_instances_name.push_back(filename);
}

void Trainer::class_stats (std::string filename)
{
	//EIGEN_ALIGN16 Eigen::Matrix3f cov_mat;
	Eigen::Matrix3f cov_mat;
	Eigen::Vector4f centroid;
	pcl::computeMeanAndCovarianceMatrix (*_class_cloud, cov_mat, centroid);
	
	//cout << filename << std::endl << cov_mat << std::endl << centroid << std::endl;
	_vec_classes_cov.push_back (cov_mat);
	_vec_classes_mean.push_back (centroid);
	_vec_classes_name.push_back(filename);
}

void Trainer::save_results ()
{
  std::ofstream instance_file;
  std::ofstream class_file;  
  std::string instance_path = _name + "_instance_stats.txt";
  std::string class_path = _name + "_class_stats.txt";
  instance_file.open (instance_path.c_str(), std::ios_base::app);
  class_file.open (class_path.c_str(), std::ios_base::app);

	// instances stats
	//cout << "Saving " << _vec_instances_cov.size () << " instances and " << _vec_classes_cov.size () << " classes." << endl;
  for (size_t i = 0; i< _vec_instances_cov.size (); ++i)
	  instance_file << _vec_instances_name[i]
					  				<< " " << _vec_instances_mean[i][0] << " " << _vec_instances_mean[i][1] << " " << _vec_instances_mean[i][2]
										<< " " << _vec_instances_cov[i](0,0) << " " << _vec_instances_cov[i](0,1) << " " << _vec_instances_cov[i](0,2)
										<< " " << _vec_instances_cov[i](1,0) << " " << _vec_instances_cov[i](1,1) << " " << _vec_instances_cov[i](1,2)
										<< " " << _vec_instances_cov[i](2,0) << " " << _vec_instances_cov[i](2,1) << " " << _vec_instances_cov[i](2,2)
										<< "\n";

  for (size_t i = 0; i< _vec_classes_cov.size (); ++i)
	  class_file << _vec_classes_name[i]
					  				<< " " << _vec_classes_mean[i][0] << " " << _vec_classes_mean[i][1] << " " << _vec_classes_mean[i][2] 
										<< " " << _vec_classes_cov[i](0,0) << " " << _vec_classes_cov[i](0,1) << " " << _vec_classes_cov[i](0,2)
										<< " " << _vec_classes_cov[i](1,0) << " " << _vec_classes_cov[i](1,1) << " " << _vec_classes_cov[i](1,2)
										<< " " << _vec_classes_cov[i](2,0) << " " << _vec_classes_cov[i](2,1) << " " << _vec_classes_cov[i](2,2)
										<< "\n";    
    
  instance_file.close();
  class_file.close();
}

float Trainer::get_num_lines (string filename)
{
  int number_of_lines = 0;
  std::string line;
  std::ifstream file (filename.c_str());

  while (std::getline (file, line))
	    ++number_of_lines;
      
	file.close ();
	 
  return number_of_lines;
}

float Trainer::get_num_dirs (string filepath)
{
	return std::count_if(directory_iterator(filepath),
				        				directory_iterator(),
								        bind( static_cast<bool(*)(const path&)>(is_regular_file), 
        							  bind( &directory_entry::path, boost::lambda::_1 ) ) );
}
