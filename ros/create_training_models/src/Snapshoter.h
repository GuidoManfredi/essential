#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// PLAN SEGMENTATION
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
// EUCLIDEAN CLUSTERING
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

// This class allows easyly taking colored clouds of an object. Start it,
// it will show a void visualizer. When in position for a snapshot, hit enter,
// the taken snapshot will be displayed in the visualizer. To confirm and save
// this snapshot press enter again. To discard the snapshot press escape.
// This class is intended to modelize a single object on a plane surface.
class Snapshoter
{
 public:
	Snapshoter ();
	// Saves a cloud
	void take (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
 	// Removes all unwanted parts of the snapshot. Keeps only the interest object,
 	// i.e. the one closest to the optical axis of the rgb camera.
	pcl::PointCloud<pcl::PointXYZ>::Ptr clean ();
	// Saves the current snapshot to a given path
	bool save (std::string save_path);
	// Allows to get the last snapshot at all time
	// Remove the current snapshot.
	void clear ();

	pcl::PointCloud<pcl::PointXYZ>	get_interest_cluster ();
		
 private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr
		remove_nan_inf (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
	// Remove the main plan of the cloud. In a normal situation this is the supporting
	// plan of the seen object.
	pcl::PointCloud<pcl::PointXYZ>::Ptr
		remove_main_plan (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
		segment_clusters (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr
		interest_cluster (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters);
	double min_distance_to_z (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster);
	double distance_to_z (pcl::PointXYZ pt);
	std::string get_last_part (std::string str);

	unsigned int numero_;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr snapshot_;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr object_;
	// state describe the current state of the application : are we waiting for
	// a snapshot to be taken (= 0); or are we waiting the user to acknowledge a 
	// taken snapshot (= 1).
	int state;
	
  pcl::SACSegmentation<pcl::PointXYZ> seg_;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_;
};

