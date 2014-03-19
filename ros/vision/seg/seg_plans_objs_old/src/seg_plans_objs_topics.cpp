#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/visualization/cloud_viewer.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "seg_plans_objs/PlantopSegmentation.h"
#include "seg_plans_objs/PointCloudArray.h"
#include "seg_plans_objs/ImageArray.h"

#include "PlanDetector.h"

namespace enc = sensor_msgs::image_encodings;

using namespace std;
using namespace cv;

PlanDetector segmenter;
Cloud::Ptr current_cloud (new Cloud);
Mat current_rgb;
int num_plans_requested = 0;

// Callback function for "segment_plans" service
bool segment (seg_plans_objs::SegPlans::Request  &req,
		         	seg_plans_objs::SegPlans::Response &res) {
  segmenter.segment_plans (current_cloud);
	segmenter.segment_clusters (current_cloud);
	segmenter.segment_rgb (current_rgb);
	
	seg_plans_objs::PlantopSegmentation segmentation;
	segmentation.tabletop_segmentation.table.pose.position =;
	segmentation.tabletop_segmentation.table.pose.orientation =;
	segmentation.tabletop_segmentation.table.x_min =;
	segmentation.tabletop_segmentation.table.x_max =;
	segmentation.tabletop_segmentation.table.y_min =;
	segmentation.tabletop_segmentation.table.y_max =;
	
  for (size_t i=0; i < segmenter.get_num_clusters(); ++i) {
	  sensor_msgs::PointCloud2 tmp;
	  pcl::toROSMsg(*(segmenter.get_cluster(i)), tmp);
  	segmentation.tabletop_segmentation.clusters.push_back(tmp);
  }
	
	for (size_t i=0; i < segmenter.get_num_rgbs(); ++i) {
		cv_bridge::CvImagePtr cv_ptr (new cv_bridge::CvImage);
		//cv_ptr->header = ; TODO fill the header with current image header
		cv_ptr->encoding = enc::BGR8;
		cv_ptr->image = *(segmenter.get_rgb_cluster(i));
		segmentation.tabletop_segmentation.cluster_images.array.push_back(*cv_ptr->toImageMsg());
	}	
	
	return true;
}

// Callback for pointcloud subscriber
void save_pointcloud (Cloud::Ptr msg) {
	current_cloud = msg;
	//segmenter.segment_plans (current_cloud);
	//segmenter.segment_clusters (current_cloud);
	//segmenter.segment_rgb (current_rgb);
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
  ros::ServiceServer srv_plans = n.advertiseService("/tabletop_segmentation", segment);
  //ros::Publisher pub_plans = n.advertise<seg_plans_objs::PointCloudArray>("segmented_plans", 10);
  //ros::Publisher pub_clusters = n.advertise<seg_plans_objs::PointCloudArray>("segmented_clusters", 10);
  //ros::Publisher pub_clusters_rgb = n.advertise<seg_plans_objs::ImageArray>("segmented_clusters_rgb", 10);

 	cout << "Segmentation service ready." << endl;
 	
 	ros::Rate loop_rate(10);
  //Mat tmp (640, 480, CV_8UC3);
  while (ros::ok()) {
	  ros::spinOnce();
		// Publish all clusters
	  /* Publish all plans
	  seg_plans_objs::PointCloudArray plans;
	  for (size_t i=0; i < segmenter.get_num_plans(); ++i) {
		  sensor_msgs::PointCloud2 tmp;
 		  pcl::toROSMsg(*(segmenter.get_plan(i)), tmp);
	  	plans.clusters.push_back(tmp);
	  }
	  pub_plans.publish(plans);
	  */
	  // publish only prefered plan
	  /*
	  if ( segmenter.get_num_plans() != 0 ) {
		  seg_plans_objs::PointCloudArray plans;
			sensor_msgs::PointCloud2 tmp;
 			pcl::toROSMsg(*(segmenter.get_prefered_plan()), tmp);
		  plans.clusters.push_back(tmp);
   		pub_plans.publish(plans);
  	}
  	
  	seg_plans_objs::PointCloudArray objects;
	  for (size_t i=0; i < segmenter.get_num_clusters(); ++i) {
  	  sensor_msgs::PointCloud2 tmp;
		  pcl::toROSMsg(*(segmenter.get_cluster(i)), tmp);
	  	objects.clusters.push_back(tmp);
	  }
  	pub_clusters.publish(objects);
  	
	  seg_plans_objs::ImageArray images;
		for (size_t i=0; i < segmenter.get_num_rgbs(); ++i) {
			cv_bridge::CvImagePtr cv_ptr (new cv_bridge::CvImage);
			//cv_ptr->header = ; TODO fill the header with current image header
			cv_ptr->encoding = enc::BGR8;
			cv_ptr->image = *(segmenter.get_rgb_cluster(i));
			images.array.push_back(*cv_ptr->toImageMsg());
			//imshow("pipo", *segmenter.get_rgb_cluster(i));
			//waitKey(0);
		}
		pub_clusters_rgb.publish(images);
		*/
  	
	  loop_rate.sleep();
  }

  return 0;
}
