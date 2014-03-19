#include <iostream>
// ROS
#include <ros/ros.h>
#include <pcl/ros/conversions.h>
// PCL
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/keyboard_event.h>

#include "Snapshoter.h"

// TODO Test on the ground, to make sure main plan is floor plan.
//			Regler problem du RGB : on veut une cloud XYZRGB a la fin, mais les traitement
//			de cleaning ne sont possible que sur XYZ. Transformer en cloud XYZ pour
//			le debut des traitement de cleaning. Dans segment_clusters recuperer les
//			indices sur l'image XYZRGB.
using namespace std;
using namespace pcl;

pcl::visualization::CloudViewer cloud_viewer ("Snapshot viewer");
Snapshoter snapshot;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud<pcl::PointXYZ>);
int state = 0;
string save_path = ".";

void cloud_cb (const sensor_msgs::PointCloud2::ConstPtr& msg) {
  pcl::fromROSMsg (*msg, *cloud);
}
// Handles the keyboard for two keys : enter, first to take the snapshot and 
// then to acknowledge a taken snapshot and save it; escape, to discard this
// snapshot and take a new one.
void keyboard_cb (const pcl::visualization::KeyboardEvent& event, void* cookie){
  if (event.getKeySym() == "s" && event.keyDown()) { // enter key
  	// waiting for a snapshot
		if (state == 0) {
			cout << "Taking snapshot..." << endl;
			snapshot.take (cloud);
			object = snapshot.clean ();
			state = 1;
			cout << "...done." << endl;
		}
		else if (state == 1) {
			cout << "Saving snapshot..." << endl;
			snapshot.save (save_path);
			state = 0;
			cout << "...done" << endl;
			object->clear();
		}
  }
  else if (event.getKeySym() == "d" && event.keyDown()) { // escape key
  	snapshot.clear ();
  	object->clear();
 	  cout << "Snapshot cleared." << endl;
  }
}

int main (int argc, char * argv[]) {
	if (argc != 3) {
		cout << "Usage : snapshot cloud_topic path_to_save_directory" << endl;
	}
	save_path = argv[2];
	
	// ROS
	ros::init(argc, argv, "snapshot");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe(argv[1], 10, cloud_cb);
	// PCL
	cloud_viewer.registerKeyboardCallback (keyboard_cb);
	while (!cloud_viewer.wasStopped(1))
  {
  	if (!object->empty())
  	 	cloud_viewer.showCloud (object);
  	else if (!cloud->empty())
  		cloud_viewer.showCloud (cloud);
  		
		ros::spinOnce();
  }
	//ros::spin();

	return 0;
}
