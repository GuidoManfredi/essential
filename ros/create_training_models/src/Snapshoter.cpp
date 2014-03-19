#include "Snapshoter.h"

using namespace std;
using namespace pcl;

Snapshoter::Snapshoter () : snapshot_ (new pcl::PointCloud<pcl::PointXYZ>){
  seg_.setOptimizeCoefficients (true);
  seg_.setModelType (pcl::SACMODEL_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setMaxIterations (100);
  seg_.setDistanceThreshold (0.02);
  
  ec_.setClusterTolerance (0.02); // 2cm
  ec_.setMinClusterSize (100);
  ec_.setMaxClusterSize (50000);
}

void Snapshoter::take (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	if (cloud->empty())
		cout << "Warning : took empty snapshot !" << endl;
	*snapshot_ = *cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Snapshoter::clean () {
	cout << "Removing NaN and Inf..." << endl;
	snapshot_ = remove_nan_inf (snapshot_);
	cout << "Removing main plan..." << endl;
	snapshot_ = remove_main_plan (snapshot_);
	cout << "Segmenting clusters..." << endl;
	clusters_ = segment_clusters (snapshot_);
	cout << "Isolating interest cluster..." << endl;
	object_ = interest_cluster (clusters_);
	return object_;
}

bool Snapshoter::save (std::string save_path) {
	// retrieve last part of path (supposedly directory name)
	stringstream file_path;
	file_path << save_path << get_last_part(save_path) << numero_ << ".pcd";
	cout << "Saving to " << file_path.str() << endl;
	if (pcl::io::savePCDFileASCII(file_path.str(), *object_) ) { // error
		return true;
	}
	else { // all went well
		++numero_;
		return false;
	}
}

void Snapshoter::clear () {
	snapshot_->clear ();
	clusters_.clear ();
	object_->clear ();
}
////////////////////////////////////////////////////////////////////////////////
//  PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr
	Snapshoter::remove_nan_inf (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr result (new pcl::PointCloud<pcl::PointXYZ>);
	vector<int> indices;
	pcl::removeNaNFromPointCloud (*cloud, *result, indices);
	return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
	Snapshoter::remove_main_plan (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr result (new pcl::PointCloud<pcl::PointXYZ>);
  // Create the segmentation object for the planar model and set all the parameters
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

	// Segment the largest planar component from the cloud
  seg_.setInputCloud (cloud);
  seg_.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0) {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  }
  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*result);
  return result;
}

vector<PointCloud<PointXYZ>::Ptr>	Snapshoter::segment_clusters (PointCloud<PointXYZ>::ConstPtr cloud) {
	vector<PointCloud<PointXYZ>::Ptr> clusters;
	// Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  ec_.setSearchMethod (tree);
  ec_.setInputCloud (cloud);
  ec_.extract (cluster_indices);
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud->points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    clusters.push_back (cloud_cluster);
  }
  cout << "Found " << clusters.size () << " clusters." << endl;
  return clusters;
}

PointCloud<PointXYZ>::Ptr Snapshoter::interest_cluster (vector<PointCloud<PointXYZ>::Ptr> clusters) {
	unsigned int min_idx = -1;
	double min_dist = 100.0;
	for (size_t i = 0; i < clusters.size (); ++i) {
		double dist = min_distance_to_z (clusters[i]);
		if (dist < min_dist) {
			min_idx = i;
			min_dist = dist;
		}
	}
	return clusters[min_idx];
}

double Snapshoter::min_distance_to_z (PointCloud<PointXYZ>::Ptr cluster) {
	double min_dist = 100.0;
	for (size_t i = 0; i < cluster->points.size(); ++i) {
		double dist  = distance_to_z (cluster->points[i]);
		if ( dist < min_dist ) {
			min_dist = dist;
		}
	}
	return min_dist;
}

double Snapshoter::distance_to_z (PointXYZ pt) {
	return sqrt (pt.x*pt.x + pt.y*pt.y);
}

string Snapshoter::get_last_part (string str) {
	size_t n = str.size();
	if (str[n-1] == '/')
		str.erase(n-1);
	n = str.size(); // take new size if changed

	unsigned int last=0;
	for (unsigned int i = 0 ; i < str.size(); i++)
  {
  	if (str[i] == '/')
  		last = i;
  }
  
	return str.substr (last, n);
}
