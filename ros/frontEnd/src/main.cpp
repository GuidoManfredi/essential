#include <ros/ros.h>
#include <assert.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "FrontEnd.h"
#include "frontEnd/CloudArray.h"

using namespace std;
using namespace cv;
using namespace pcl;

Mat img, depth;

// G2O test_backend.cpp

/** TODO
 *
 * Problem quand prend 2 keyframe d'affilé : perdu.
 * Corriger sens de rotation du repere
 *
**/
/*
3 sources d'erreur : 
    Utiliser camera/depth_registered/points
             et non
             camera/depth/points
    Calibrer correctement la camera rgb
    Verifier que openni_launch recupere les bon parametres intrinseques pour
    sa rectification.
*/
void cloud_callback(const sensor_msgs::PointCloud2& msg) {
	PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);
	fromROSMsg (msg, *cloud);
	unsigned int h = cloud->height;
	unsigned int w = cloud->width;
	Mat tmp_depth (h, w, CV_32FC3);
	Mat tmp_rgb (h, w, CV_8UC3);
	for (size_t y=0; y<h; ++y) {
		for (size_t x=0; x<w; ++x) {
			PointXYZRGB pt;
			pt = cloud->at(x,y);
			tmp_depth.at<Vec3f>(y, x)[0] = pt.x;
			tmp_depth.at<Vec3f>(y, x)[1] = pt.y;
			tmp_depth.at<Vec3f>(y, x)[2] = pt.z;
			int rgb = *reinterpret_cast<int*>(&pt.rgb);
            tmp_rgb.at<Vec3b>(y, x)[0] = (rgb & 0xff); // B
            tmp_rgb.at<Vec3b>(y, x)[1] = ((rgb >> 8) & 0xff); // G
            tmp_rgb.at<Vec3b>(y, x)[2] = ((rgb >> 16) & 0xff); // R
		}
	}
	depth = tmp_depth;
	img = tmp_rgb;
	//imshow ("depth", depth);
	//imshow ("rgb", tmp_rgb);
	//waitKey(1);
}

geometry_msgs::Pose Mat2Pose (Mat P) {
	Eigen::Matrix3d R;
	R << P.at<double>(0,0), P.at<double>(0,1), P.at<double>(0,2),
		 P.at<double>(1,0), P.at<double>(1,1), P.at<double>(1,2),
		 P.at<double>(2,0), P.at<double>(2,1), P.at<double>(2,2);
	Eigen::Quaterniond q(R);
	
	geometry_msgs::Pose pose;
	pose.position.x = P.at<double>(0, 3);
	pose.position.y = P.at<double>(1, 3);
	pose.position.z = P.at<double>(2, 3);
	pose.orientation.x = q.x();
	pose.orientation.y = q.y();
	pose.orientation.z = q.z();
	pose.orientation.w = q.w();
	return pose;
}

geometry_msgs::PoseArray vectorMat2PoseArray (vector<Mat> P) {
    geometry_msgs::PoseArray keyframes;
    keyframes.header.stamp = ros::Time::now();
    for (size_t i = 0; i < P.size(); ++i) {
        keyframes.poses.push_back (Mat2Pose (P[i]));
        
    }
    return keyframes;
}

geometry_msgs::PoseStamped Mat2PoseStamped (Mat P) {
	Eigen::Matrix3d R;
	R << P.at<double>(0,0), P.at<double>(0,1), P.at<double>(0,2),
		 P.at<double>(1,0), P.at<double>(1,1), P.at<double>(1,2),
		 P.at<double>(2,0), P.at<double>(2,1), P.at<double>(2,2);
	Eigen::Quaterniond q(R);
	
	geometry_msgs::PoseStamped camera;
	camera.header.stamp = ros::Time::now();
	camera.pose.position.x = P.at<double>(0, 3);
	camera.pose.position.y = P.at<double>(1, 3);
	camera.pose.position.z = P.at<double>(2, 3);
	camera.pose.orientation.x = q.x();
	camera.pose.orientation.y = q.y();
	camera.pose.orientation.z = q.z();
	camera.pose.orientation.w = q.w();
	return camera;
}

vector<sensor_msgs::PointCloud2> pcl2ros (vector<pcl::PointCloud<pcl::PointXYZ> > clouds) {
    vector<sensor_msgs::PointCloud2> ros_clouds;
    for ( size_t i = 0; i < clouds.size(); ++i ) {
        sensor_msgs::PointCloud2 ros;
        pcl::toROSMsg(clouds[i], ros);
        ros_clouds.push_back(ros);
    }
    return ros_clouds;
}

int main(int argc, char** argv)
{
	assert (argc==6 && "Usage : frontEnd in_image_topic in_cloud_topic out_pose_topic out_keyframe_pose_topic out_pointcloud_topic");
	
	ros::init(argc, argv, "frontEnd");
	ros::NodeHandle n;

	ros::Subscriber cloud_subscriber = n.subscribe(argv[2], 1, cloud_callback);
	ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>(argv[3], 1);
	ros::Publisher keyframe_publisher = n.advertise<geometry_msgs::PoseArray>(argv[4], 1);
	ros::Publisher points_publisher = n.advertise<frontEnd::CloudArray>(argv[5], 1);

	string calibration_path ("/home/gmanfred/.ros/camera_info/my_xtion.yml");
	FrontEnd fend (calibration_path);

	ros::Rate loop_rate (10);
	while (ros::ok()) {
		ros::spinOnce ();
		fend.process (img, depth);
        // Publish current pose		
		Mat P = fend.getLastPose();
		if (P.data) { // if there is frame
    		geometry_msgs::PoseStamped msg = Mat2PoseStamped (P);
	    	pose_pub.publish (msg);
	    } else {
	        cout << "No pose." << endl;
	    }
	    // keyframes
	    vector<Mat> kf = fend.getKeyframePoses();
	    if (kf.size() != 0) {
	        geometry_msgs::PoseArray pose_array = vectorMat2PoseArray (kf);
	        keyframe_publisher.publish (pose_array);
	    } else {
	        cout << "No keyframes." << endl;
	    }
        // pointclouds        
        vector<pcl::PointCloud<pcl::PointXYZ> > clouds = fend.getPoints();
        if (clouds.size() != 0) {            
	        frontEnd::CloudArray cloud_array;
	        cloud_array.clouds = pcl2ros (clouds);
	        points_publisher.publish (cloud_array);
	    } else {
	        cout << "No point3D." << endl;
	    }
		loop_rate.sleep ();
	}

  return 0;
}

