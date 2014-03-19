#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv_display/LocaPose.h"
#include "Visualizer.h"

/*
*** TEST line ***
rostopic pub pose opencv_display/LocaPose -- '0.904649' '-0.395126' '0.159642' '0.378819' '-0.243246' '0.574008' '-0.725955' '0.0737445' '0.195208' '0.71721' '0.668957' '-4.09743'
rostopic pub pose opencv_display/LocaPose -- '1.0' '0.0' '0.0' '0.378819' '0.0' '1.0' '0.0' '0.0737445' '0.0' '0.0' '1.0' '-4.09743'
*/
using namespace std;
namespace enc = sensor_msgs::image_encodings;
using namespace cv;

Visualizer* display;

void image_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
    }
    display->updateBackground(cv_ptr->image);
}

void localisation_callback(const opencv_display::LocaPose::ConstPtr& poses_msg) {
	Mat T = Mat::eye (4, 4, CV_64F);
	T.at<double>(0,0) = poses_msg->t00;	T.at<double>(0,1) = poses_msg->t01;	T.at<double>(0,2) = poses_msg->t02;  T.at<double>(0, 3) = poses_msg->t03;
	T.at<double>(1,0) = poses_msg->t10;	T.at<double>(1,1) = poses_msg->t11;	T.at<double>(1,2) = poses_msg->t12;  T.at<double>(1, 3) = poses_msg->t13;
	T.at<double>(2,0) = poses_msg->t20;	T.at<double>(2,1) = poses_msg->t21;	T.at<double>(2,2) = poses_msg->t22;  T.at<double>(2, 3) = poses_msg->t23;
	cout << T << endl;
	display->updatePose(T);
}
/*
void localisation_callback(const opencv_display::LocaPose::ConstPtr& poses_msg) {
	Mat T = Mat::eye (4, 4, CV_64F);
	T.at<float>(0,0) = poses_msg->t00;	T.at<float>(0,1) = poses_msg->t01;	T.at<float>(0,2) = poses_msg->t02;  T.at<float>(0, 3) = poses_msg->t03;
	T.at<float>(1,0) = poses_msg->t10;	T.at<float>(1,1) = poses_msg->t11;	T.at<float>(1,2) = poses_msg->t12;  T.at<float>(1, 3) = poses_msg->t13;
	T.at<float>(2,0) = poses_msg->t20;	T.at<float>(2,1) = poses_msg->t21;	T.at<float>(2,2) = poses_msg->t22;  T.at<float>(2, 3) = poses_msg->t23;
    cout << T << endl;
	display->updatePose(T);
}
*/

int main (int argc, char** argv) {
	assert (argc == 3 && "Usage : display_poses image_topic locations_topic");

	ros::init(argc, argv, "opencv_display_poses");
	ros::NodeHandle n;
	// Subscribe to relevant information
	ros::Subscriber image_subscriber = n.subscribe(argv[1], 1, image_callback);
	ros::Subscriber localisation_subscriber = n.subscribe(argv[2], 1, localisation_callback);
	// Load intrinsic parameters
	Mat K(3,3,CV_32F);
    Mat d(5,1,CV_32F);
    FileStorage r_fs;
    r_fs.open ("/home/gmanfred/.ros/camera_info/my_xtion.yml", cv::FileStorage::READ);
    r_fs["camera_matrix"]>>K;
    r_fs["distortion_coefficients"]>>d;
    r_fs.release ();
    // Init display
    display = new Visualizer ("Display poses", Size(640, 480), K);
  
	ros::Rate loop_rate(100);
	while (ros::ok()) {
	    ros::spinOnce();

		display->updateWindow();

	    cv::waitKey(1);
	    loop_rate.sleep();
	}

    return 0;  
}

