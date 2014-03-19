#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>

#include <geometry_msgs/Vector3.h>

#include "segment_plans_objects/PlantopSegmentation.h"
//#include "reco_3d/OrientedBoundingBoxRecognition.h"
#include "reco_3d/IterativeClosestPointRecognition.h"

using namespace std;
using namespace pcl;
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

visualization::PCLVisualizer cloud_viewer("PCL Display");
ros::ServiceClient srv_segment;
ros::ServiceClient srv_recognize;

vector<string> used_names;
vector<int> idx;

segment_plans_objects::PlantopSegmentation segmentation;
//reco_3d::OrientedBoundingBoxRecognition recognition;
reco_3d::IterativeClosestPointRecognition recognition;

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

void add_cube (Eigen::Vector3f translation, Eigen::Quaternionf rotation,
               double width, double height, double depth, string name) {
    pcl::ModelCoefficients coeff_cube;
    coeff_cube.values.resize(10);
    coeff_cube.values[0] = translation.x();
    coeff_cube.values[1] = translation.y();
    coeff_cube.values[2] = translation.z();
    coeff_cube.values[3] = rotation.x();
    coeff_cube.values[4] = rotation.y();
    coeff_cube.values[5] = rotation.z();
    coeff_cube.values[6] = rotation.w();
    coeff_cube.values[7] = width;
    coeff_cube.values[8] = height;
    coeff_cube.values[9] = depth;
    cloud_viewer.addCube(coeff_cube, name);
}
/*
void add_table (tabletop_object_detector::Table table) {
	// clean previous table
	cloud_viewer.removeShape("table");
	Eigen::Vector3f t;
	t(0) = table.pose.pose.position.x;
	t(1) = table.pose.pose.position.y;
	t(2) = table.pose.pose.position.z;
	Eigen::Quaternionf q;
	q.x() = table.pose.pose.orientation.x;
	q.y() = table.pose.pose.orientation.y;
	q.z() = table.pose.pose.orientation.z;
	q.w() = table.pose.pose.orientation.w;
	float w, h, d;
	w = table.x_max - table.x_min;
	h = table.y_max - table.y_min;
	d = 0.01;
	add_cube (t, q, w, h, d, "table");
}
*/
void add_obb (geometry_msgs::PoseStamped pose, geometry_msgs::Vector3 box_dims,
							string name, int idx) {
	ostringstream shape_name, text;
	shape_name << name << "_shape" << idx;
	text << name << idx;
	//cloud_viewer.removeShape(shape_name.str());
	//cloud_viewer.removeText3D(text.str());
	Eigen::Vector3f t;
	t(0) = pose.pose.position.x;
	t(1) = pose.pose.position.y;
	t(2) = pose.pose.position.z;
	Eigen::Quaternionf q;
	q.x() = pose.pose.orientation.x;
	q.y() = pose.pose.orientation.y;
	q.z() = pose.pose.orientation.z;
	q.w() = pose.pose.orientation.w;
	cloud_viewer.addText3D (text.str(), pcl::PointXYZ(t(0), t(1), t(2)-box_dims.z), 0.01);
	//cloud_viewer.addText3D (text, pcl::PointXYZ(t(0), t(1), t(2)), 0.01);
	add_cube (t, q, box_dims.x, box_dims.y, box_dims.z, shape_name.str());
}

void add_coordinate_frame (geometry_msgs::PoseStamped pose) {
	Eigen::Vector3f t;
	t(0) = pose.pose.position.x;
	t(1) = pose.pose.position.y;
	t(2) = pose.pose.position.z;
	Eigen::Quaternionf q;
	q.x() = pose.pose.orientation.x;
	q.y() = pose.pose.orientation.y;
	q.z() = pose.pose.orientation.z;
	q.w() = pose.pose.orientation.w;
	Eigen::Matrix3f R = q.toRotationMatrix();
	Eigen::Vector3f r = R.eulerAngles(0, 1, 2);
	Eigen::Affine3f A;
	pcl::getTransformation (t(0), t(1), t(2),
													r(0), r(1), r(2), A);
	cloud_viewer.addCoordinateSystem (0.1, A);
}

void add_coordinate_frame (Eigen::Vector3f translation,
													 Eigen::Vector3f rotation) {
	Eigen::Affine3f t;
	pcl::getTransformation (translation(0), translation(1), translation(2),
													rotation(0), rotation(1), rotation(2), t);
	cloud_viewer.addCoordinateSystem (0.1, t);
}

void add_clusters (vector<sensor_msgs::PointCloud2> clusters) {
	Cloud::Ptr cloud (new Cloud);
	std::ostringstream cloud_name;
	//cout << "Receiving " << msg->clusters.size() << " clusters/plans" << endl;
	for (size_t i=0; i<clusters.size(); ++i) {
		cloud_name.str("");
	  pcl::fromROSMsg(clusters[i], *cloud);
		cloud_name << "cloud" << i;
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler (cloud, r[i], g[i], b[i]);
		if (!cloud_viewer.updatePointCloud (cloud, handler, cloud_name.str()))
			cloud_viewer.addPointCloud (cloud, handler, cloud_name.str());
	}
}

void one_cloud_cb(const Cloud::ConstPtr& msg) {
	if (!cloud_viewer.updatePointCloud (msg, "single_cloud"))
		cloud_viewer.addPointCloud (msg, "single_cloud");
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void) {
  if (event.getKeySym () == "s" && event.keyDown ()) {
    ROS_INFO("Calling segmentation service");

		segmentation.request.num_plans_requested = 1;
		// call segmentation service
		if (srv_segment.call(segmentation)) {
		  if (segmentation.response.result == 4) { // success
			  cloud_viewer.removeAllShapes ();
		  	//add_table (segmentation.response.table);
		  	add_clusters (segmentation.response.clusters);
		  	
	  		std::ostringstream obb_name;
		  }
		} 
		else
		 	ROS_ERROR("Segmentation failed with error %d", segmentation.response.result);
	}
  else if (event.getKeySym () == "r" && event.keyDown ()) {
	  ROS_INFO("Calling recognition service");
	  if (segmentation.response.result == 4) { // success
			cloud_viewer.removeAllShapes ();
		  //add_table (segmentation.response.table);
		  add_clusters (segmentation.response.clusters);
			// for each cluster call recognition service
			//for (size_t i=0; i<segmentation.response.clusters.size(); ++i) {
				recognition.request.cluster = segmentation.response.clusters[2];
				if (srv_recognize.call (recognition)) {
					/*
					add_obb (recognition.response.pose,
									 recognition.response.box_dims,
									 recognition.response.names[0],
									 i);
					*/
					add_coordinate_frame (recognition.response.pose);
				}
				else
					ROS_ERROR("Failed to call recognition service");
			//}
		}
	}
}

int main (int argc, char** argv) {
	init_colors ();

	ros::init(argc, argv, "pcl_display");
	ROS_INFO ("Press \"s\" for segmentation and \"r\" for recognition");
	ros::NodeHandle n;
	string SEGMENTATION_SERVICE = argv[1];
	string RECOGNITION_SERVICE = argv[2];
	srv_segment = n.serviceClient<segment_plans_objects::PlantopSegmentation>(SEGMENTATION_SERVICE);
	srv_recognize = n.serviceClient<reco_3d::IterativeClosestPointRecognition>(RECOGNITION_SERVICE);
	//ros::Subscriber sub_clouds = n.subscribe(argv[1], 1, one_cloud_cb);

	cloud_viewer.registerKeyboardCallback (keyboardEventOccurred, (void*)&cloud_viewer);
	cloud_viewer.setBackgroundColor (0, 0, 0);
	cloud_viewer.initCameraParameters ();
	cloud_viewer.addCoordinateSystem (0.5);
	
	ros::Rate loop_rate(10);
	while (ros::ok() && !cloud_viewer.wasStopped()) {
    cloud_viewer.spinOnce(1);
		ros::spinOnce();
		loop_rate.sleep ();
	}

	return 0;
}
