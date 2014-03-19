#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include "frontEnd/CloudArray.h"

using namespace std;
using namespace pcl;
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

visualization::PCLVisualizer cloud_viewer("SLAM display");

// We suppose there won't be more than 256 clusters
double r[256];
double g[256];
double b[256];

void init_colors () {
	for (int i=0; i<256; ++i) {
		r[i] = rand()%255;
		g[i] = rand()%255;
		b[i] = rand()%255;
	}
}

void addCoordinateFrame (geometry_msgs::Pose pose) {
	cloud_viewer.addCoordinateSystem (0.5);
	Eigen::Vector3f t;
	t(0) = pose.position.x;
	t(1) = pose.position.y;
	t(2) = -pose.position.z; // PCL seems to have a strange Z axis
	Eigen::Quaternionf q;
	q.x() = pose.orientation.x;
	q.y() = pose.orientation.y;
	q.z() = pose.orientation.z;
	q.w() = pose.orientation.w;
	Eigen::Matrix3f R = q.toRotationMatrix();
	Eigen::Vector3f r = R.eulerAngles(0, 1, 2);
	Eigen::Affine3f A;
	pcl::getTransformation (t(0), t(1), t(2), r(0), r(1), r(2), A);
	cloud_viewer.addCoordinateSystem (0.2, A);
}

void addCoordinateFrame (geometry_msgs::PoseStamped pose_stamped) {
    addCoordinateFrame (pose_stamped.pose);
}

void addCluster (sensor_msgs::PointCloud2 cluster, int n) {    
	Cloud::Ptr cloud (new Cloud);
	pcl::fromROSMsg(cluster, *cloud);

	std::ostringstream cloud_name;
	cloud_name.str("");
	cloud_name << "cloud" << n;
	
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler (cloud, r[n], g[n], b[n]);
	cloud_viewer.addPointCloud (cloud, handler, cloud_name.str());
}

void addClusters (vector<sensor_msgs::PointCloud2> clusters) {
    cloud_viewer.removeAllPointClouds ();
    for (size_t i=0; i<clusters.size(); ++i) {
        addCluster (clusters[i], i);
    }
}

void pose_callback (geometry_msgs::PoseStamped msg) {
    cloud_viewer.removeCoordinateSystem (0);
	addCoordinateFrame (msg);
}

void keyframe_callback (geometry_msgs::PoseArray msg) {
    /*
    for (size_t i = 0; i < msg.poses.size(); ++i) {       
   	    cloud_viewer.removeCoordinateSystem (0);
	}
    for (size_t i = 0; i < msg.poses.size(); ++i) {
    	addCoordinateFrame (msg.poses[i]);
	}
    */
}

void points_callback (frontEnd::CloudArray msg) {
    addClusters (msg.clouds);
}

int main (int argc, char** argv) {
	assert (argc == 4 && "Usage : display_slam in_pose_topic in_keyframe_pose_topic in_pointclouds");

	ros::init(argc, argv, "pcl_display");
	ros::NodeHandle n;

	ros::Subscriber pose_sub = n.subscribe(argv[1], 1, pose_callback);
	ros::Subscriber keyframe_sub = n.subscribe(argv[2], 1, keyframe_callback);
	ros::Subscriber points_sub = n.subscribe(argv[3], 1, points_callback);

	init_colors ();
	cloud_viewer.setBackgroundColor (0, 0, 0);
	cloud_viewer.initCameraParameters ();
	cloud_viewer.addCoordinateSystem (1.0);
	
	ros::Rate loop_rate(10);
	while (ros::ok() && !cloud_viewer.wasStopped()) {
        //cloud_viewer.removeAllShapes ();
        cloud_viewer.spinOnce(1);
		ros::spinOnce();
		loop_rate.sleep ();
	}

	return 0;
}
