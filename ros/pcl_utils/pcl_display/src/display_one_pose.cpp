#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace pcl;

visualization::PCLVisualizer cloud_viewer("SLAM display");

void addCoordinateFrame (geometry_msgs::Pose pose) {
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
	//pcl::getTransformation (t(0), t(1), t(2), r(0), r(1), r(2), A);
	pcl::getTransformation (0, 0, 0, r(0), r(1), r(2), A); // Pour observer le repere à l'origine
	cloud_viewer.addCoordinateSystem (0.2, A);
}

void addCoordinateFrame (geometry_msgs::PoseStamped pose_stamped) {
    addCoordinateFrame (pose_stamped.pose);
}

void pose_callback (geometry_msgs::PoseStamped msg) {
    cloud_viewer.removeCoordinateSystem (0);
	addCoordinateFrame (msg);
}

int main (int argc, char** argv) {
	assert (argc == 2 && "Usage : display_slam in_pose_topic");

	ros::init(argc, argv, "pcl_display");
	ros::NodeHandle n;

	ros::Subscriber pose_sub = n.subscribe(argv[1], 1, pose_callback);

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
