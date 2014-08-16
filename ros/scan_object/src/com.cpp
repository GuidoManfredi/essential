#include <ros/ros.h>
#include <assert.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "COM.h"

using namespace std;
using namespace cv;
using namespace pcl;

Mat img, depth;

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

int main(int argc, char** argv)
{
	assert (argc==6 && "Usage : frontEnd in_image_topic in_cloud_topic out_pointcloud_topic");
	
	ros::init(argc, argv, "frontEnd");
	ros::NodeHandle n;

	ros::Subscriber cloud_subscriber = n.subscribe(argv[2], 1, cloud_callback);
	ros::Publisher points_publisher = n.advertise<frontEnd::CloudArray>(argv[3], 1);

	string calibration_path ("/home/gmanfred/.ros/camera_info/my_xtion.yml");
	COM com (calibration_path);

	ros::Rate loop_rate (10);
	while (ros::ok()) {
		ros::spinOnce ();
		com.addFrame (img, depth, );
		loop_rate.sleep ();
	}

  return 0;
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

