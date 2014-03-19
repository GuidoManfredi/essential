#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/visualization/cloud_viewer.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "seg_plans_objs/PlantopSegmentation.h"
#include "PlanDetector.h"

namespace enc = sensor_msgs::image_encodings;

using namespace std;
using namespace cv;

PlanDetector segmenter;
Cloud::Ptr current_cloud (new Cloud);
Mat current_rgb;
int num_plans_requested = 0;

// Callback function for "segment_plans" service
bool segment (seg_plans_objs::PlantopSegmentation::Request  &req,
		         	seg_plans_objs::PlantopSegmentation::Response &res) {		         	
	if (current_cloud->empty()) {
		res.result = 1;
		cout << "Error : seg_plans_objs : no current cloud" << endl;
		return false;
	}
	
	res.result = 4;
  if (segmenter.segment_plans (current_cloud) == 0)
		res.result = 2;
	else if (segmenter.segment_clusters (current_cloud) == 0)
		res.result = 3;
	else if (segmenter.segment_rgb (current_rgb) == 0)
		res.result = 3;

	// TODO Remplir le header pour la pose de la table
	Eigen::Vector3f t = segmenter.get_prefered_plan_translation ();
	res.table.pose.header = current_cloud->header;
	res.table.pose.pose.position.x = t(0);
	res.table.pose.pose.position.y = t(1);
	res.table.pose.pose.position.z = t(2);
	Eigen::Quaternionf q = segmenter.get_prefered_plan_orientation ();
	// Be carefull here, eigen quaternion ordering is : w, x, y, z
	// while ROS is : x, y, z, w
	res.table.pose.pose.orientation.x = q.x();
	res.table.pose.pose.orientation.y = q.y();
	res.table.pose.pose.orientation.z = q.z();
	res.table.pose.pose.orientation.w = q.w();
	Eigen::Vector4f min, max;
	min = segmenter.get_prefered_plan_min ();
	max = segmenter.get_prefered_plan_max ();
	res.table.x_min = min(0);
	res.table.x_max = max(0);
	res.table.y_min = min(1);
	res.table.y_max = max(1);

	for (size_t i=0; i < segmenter.get_num_clusters(); ++i) {
		sensor_msgs::PointCloud2 tmp;
		pcl::toROSMsg(*(segmenter.get_cluster(i)), tmp);
		res.clusters.push_back(tmp);
	}

	for (size_t i=0; i < segmenter.get_num_rgbs(); ++i) {
		cv_bridge::CvImagePtr cv_ptr (new cv_bridge::CvImage);
		//cv_ptr->header = ; TODO fill the header with current image header
		cv_ptr->encoding = enc::BGR8;
		cv_ptr->image = *(segmenter.get_rgb_cluster(i));
		res.cluster_images.push_back(*cv_ptr->toImageMsg());
	}
	return true;
}

// Callback for pointcloud subscriber
void save_pointcloud (Cloud::Ptr msg) {
	current_cloud = msg;
}

void save_rgb (const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
	current_rgb = cv_ptr->image;
}

int main(int argc, char **argv) {
	if (argc != 3) {
		std::cout << "Usage: seg_plans_objs in_pointcloud_topic in_rgb_topic" << std::endl;
		return 1;
	}
  ros::init(argc, argv, "segmentation_node");
  ros::NodeHandle n;
	// Input
  ros::Subscriber sub_pointcloud = n.subscribe(argv[1], 10, save_pointcloud);
  ros::Subscriber sub_rgb = n.subscribe(argv[2], 10, save_rgb);
	// Output
  ros::ServiceServer srv_plans = n.advertiseService("/segmentation_plantop", segment);
 	cout << "Segmentation service ready." << endl;
 	
 	ros::Rate loop_rate(10);
  while (ros::ok()) {
	  ros::spinOnce();
	  loop_rate.sleep();
  }

  return 0;
}
