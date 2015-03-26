#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>

using namespace std;
using namespace pcl;
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

void add_coordinate_frame (Eigen::Vector3f translation,
													 Eigen::Vector3f rotation) {
	Eigen::Affine3f t;
	pcl::getTransformation (translation(0), translation(1), translation(2),
													rotation(0), rotation(1), rotation(2), t);
	cloud_viewer.addCoordinateSystem (0.1, t);
}

void one_cloud_cb(const Cloud::ConstPtr& msg) {
	if (!cloud_viewer.updatePointCloud (msg, "single_cloud"))
		cloud_viewer.addPointCloud (msg, "single_cloud");
}

int main (int argc, char** argv) {
	init_colors ();

	ros::init(argc, argv, "pcl_display");
	ros::NodeHandle n;
	ros::Subscriber sub_clouds = n.subscribe(argv[1], 1, one_cloud_cb);

	cloud_viewer.setBackgroundColor (0, 0, 0);
	cloud_viewer.initCameraParameters ();
	
	ros::Rate loop_rate(10);
	while (ros::ok() && !cloud_viewer.wasStopped()) {
	    cloud_viewer.spinOnce(1);
		ros::spinOnce();
		loop_rate.sleep ();
	}

	return 0;
}
