#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/TransformStamped.h"

#include <opencv2/highgui/highgui.hpp>

#include "../hom/HOM.h"

HOD detecter;
vector<Mat> Ps;

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

opencv_display::LocaPose mat2msg (Mat P);

void image_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    int start = cv::getTickCount();
    detecter.process (cv_ptr->image, Ps);
    int end = cv::getTickCount();
    float time_period = 1 / cv::getTickFrequency();
    ROS_INFO("Procesing time: %f s.", (end - start) * time_period);
}
// ./bin/detect /camera/depth_registered/points pose purfruit
// rosrun opencv_display display_poses /camera/rgb/image_rect_color pose
int main (int argc, char** argv) {
	assert (argc == 4 && "Usage : detect_objects in_image_topic out_pose_topic objects_list_path");
	ros::init(argc, argv, "hom");
	ros::NodeHandle n;
	ros::Subscriber cloud_subscriber = n.subscribe(argv[1], 1, image_callback);
	ros::Publisher pose_publisher = n.advertise<opencv_display::LocaPose>(argv[2], 1);

    detecter.loadIntrinsic ("/home/gmanfred/.ros/camera_info/webcam_gilgamesh_opencv.yml");
    detecter.loadFacesFromList (argv[3]);
	while (ros::ok()) {
        ros::spinOnce();
        opencv_display::LocaPose msg;
        msg = mat2msg (P);
   	    pose_publisher.publish(msg);
    }

    return 0;  
}
/*
void mats2msg (vector<Mat> Ps,
               vector<geometry_msgs::TransformStamped> transforms) {
    geometry_msgs::TransformStamped transforms[];
    
    transformsVector3 translation
Quaternion rotation
    
    msg.t00 = P.at<float>(0, 0);    msg.t01 = P.at<float>(0, 1);    msg.t02 = P.at<float>(0, 2);    msg.t03 = P.at<float>(0, 3);
    msg.t10 = P.at<float>(1, 0);    msg.t11 = P.at<float>(1, 1);    msg.t12 = P.at<float>(1, 2);    msg.t13 = P.at<float>(1, 3);
    msg.t20 = P.at<float>(2, 0);    msg.t21 = P.at<float>(2, 1);    msg.t22 = P.at<float>(2, 2);    msg.t23 = P.at<float>(2, 3);
    return msg;
}

geometry_msgs::TransformStamped mat2transform(Mat P) {

}
*/