#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>

#include "seg_plans_objs/PlantopSegmentation.h"
#include "seg_plans_objs/PointCloudArray.h"
#include "reco_3d/OBBArray.h"

using namespace std;
using namespace pcl;
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

visualization::PCLVisualizer cloud_viewer("PCL Display");

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

void one_cloud_cb(const Cloud::ConstPtr& msg) {
	if (!cloud_viewer.updatePointCloud (msg, "single_cloud"))
		cloud_viewer.addPointCloud (msg, "single_cloud");
}

void vec_cloud_cb(const seg_plans_objs::PointCloudArray::ConstPtr& msg) {
	Cloud::Ptr cloud (new Cloud);
	std::ostringstream cloud_name;
	//double r = 255, g = 255, b = 255;
	//cout << "Receiving " << msg->clusters.size() << " clusters/plans" << endl;
	for (size_t i=0; i<msg->clusters.size(); ++i) {
		cloud_name.str("");
	  pcl::fromROSMsg(msg->clusters[i], *cloud);
		cloud_name << "cloud" << i;
		//double r = rand()%255, g = rand()%255, b = rand()%255;
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler (cloud, r[i], g[i], b[i]);
		if (!cloud_viewer.updatePointCloud (cloud, handler, cloud_name.str()))
			cloud_viewer.addPointCloud (cloud, handler, cloud_name.str());
	}
}

void vec_obb_cb(const reco_3d::OBBArray::ConstPtr& msg) {
	std::ostringstream cloud_name;
	for (size_t i=0; i<msg->array.size(); ++i) {
		cloud_name.str("");
		cloud_name << "cloud" << i;
		Eigen::Vector3f t;
		t(0) = msg->array[i].transform.pose.position.x;
		t(1) = msg->array[i].transform.pose.position.y;
		t(2) = msg->array[i].transform.pose.position.z;
		Eigen::Quaternionf r (msg->array[i].transform.pose.orientation.w,
													msg->array[i].transform.pose.orientation.x,
													msg->array[i].transform.pose.orientation.y,
													msg->array[i].transform.pose.orientation.z);
		// cleaning before adding a new one
		cloud_viewer.removeShape(cloud_name.str());
	  add_cube (t, r, msg->array[i].width, msg->array[i].height, msg->array[i].depth, cloud_name.str());
	}
}

void add_table (seg_plans_objs::PlantopSegmentation::Response res) {
	Eigen::Affine3f t;
	pcl::getTransformation (0.0152947, -0.110054, 0.820532, 2.38951, 0, 3.11069, t);
	cloud_viewer.addCoordinateSystem (0.1, t);
}

void add_clusters (seg_plans_objs::PlantopSegmentation::Response res) {
	Cloud::Ptr cloud (new Cloud);
	std::ostringstream cloud_name;
	//double r = 255, g = 255, b = 255;
	//cout << "Receiving " << msg->clusters.size() << " clusters/plans" << endl;
	for (size_t i=0; i<res.clusters.size(); ++i) {
		cloud_name.str("");
	  pcl::fromROSMsg(res->clusters[i], *cloud);
		cloud_name << "cloud" << i;
		//double r = rand()%255, g = rand()%255, b = rand()%255;
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler (cloud, r[i], g[i], b[i]);
		if (!cloud_viewer.updatePointCloud (cloud, handler, cloud_name.str()))
			cloud_viewer.addPointCloud (cloud, handler, cloud_name.str());
	}
}

int main (int argc, char** argv) {
	if (argc != 4) {
		cout << "Usage : pcl_display simple_cloud_topic vector_clouds_topic vectors_obb_topic" << endl;
		return 1;
	}
	init_colors ();
		
	ros::init(argc, argv, "pcl_display");
	ros::NodeHandle n;

	ros::ServiceClient srv_segment = n.serviceClient<seg_plans_objs::PlantopSegmentation>("/plantop_segmentation");
	seg_plans_objs::PlantopSegmentation srv;
	srv.request.num_plans_requested = 1;

  if (srv_segment.call(srv)) {
    if (srv.response.result == 4) { // success
    	
    }
  } else {
    ROS_ERROR("Failed to call service /plantop_segmentation");
    return 1;
  }
	
	//ros::Subscriber sub_clouds = n.subscribe(argv[1], 1000, one_cloud_cb);
	//ros::Subscriber sub_vec_clouds = n.subscribe(argv[2], 1000, vec_cloud_cb);
	//ros::Subscriber sub_vec_obb = n.subscribe(argv[3], 1000, vec_obb_cb);

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
