#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv_display/LocaPose.h"

#include "../pom/POD.h"

// !!! ATTENTION ! NE JAMAIS METTRE 2 FOIS LA MEME IMAGE DANS LE MODELE !!!
// si des descripteurs sont en double, lors du matching ils ne seront pas pris
// en compte par siftgpu, vas savoir pourquoi...

// ./bin/detect_object image pose lait
// rosrun opencv_publisher stream 0 webcam image
// rosrun opencv_display display_poses age pose

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

POD detecter;
Object object;
Mat P = Mat::eye(4, 4, CV_32F);

Object loadObject (char* path);
opencv_display::LocaPose mat2msg (Mat P);

void cloud_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
//	imshow ("Input", cv_ptr->image);
//    waitKey(1);
    int start = cv::getTickCount();
    P = detecter.process (cv_ptr->image);
    cout << P << endl;
    //P = P.inv();
    P.at<float>(0, 3) *= -1.0/100;
    P.at<float>(1, 3) *= -1.0/100;
    P.at<float>(2, 3) *= -1.0/100;
    //cout << P << endl;
    int end = cv::getTickCount();
    float time_period = 1 / cv::getTickFrequency();
    ROS_INFO("Procesing time: %f s.", (end - start) * time_period);
}

// ./bin/detect_model /camera/depth_registered/points pose purfruit
// rosrun opencv_display display_poses /camera/rgb/image_rect_color pose
int main (int argc, char** argv) {
	assert (argc == 4 && "Usage : detect_objects in_image_topic out_pose_topic object_name");
	ros::init(argc, argv, "pom");
	ros::NodeHandle n;
	ros::Subscriber cloud_subscriber = n.subscribe(argv[1], 1, cloud_callback);
	ros::Publisher pose_publisher = n.advertise<opencv_display::LocaPose>(argv[2], 1);

    detecter.loadIntrinsic ("/home/gmanfred/.ros/camera_info/webcam_gilgamesh_opencv.yml");
    Object object = loadObject (argv[3]);
    detecter.setObject (object);
	while (ros::ok()) {
        ros::spinOnce();
        opencv_display::LocaPose msg;
        msg = mat2msg (P);
   	    pose_publisher.publish(msg);
    }

    return 0;
}

Object loadObject (char* path) {
    string object_name (path);
    string object_file = object_name + ".yaml";
    FileStorage fs(object_file, FileStorage::READ);
    Object object;
    fs[object_name] >> object;
    if ( object.views_.empty() )
        cout << "Load object " << object_file << " failed" << endl;
    return object;
}

opencv_display::LocaPose mat2msg (Mat P) {
    opencv_display::LocaPose msg;
    msg.t00 = P.at<float>(0, 0);    msg.t01 = P.at<float>(0, 1);    msg.t02 = P.at<float>(0, 2);    msg.t03 = P.at<float>(0, 3);
    msg.t10 = P.at<float>(1, 0);    msg.t11 = P.at<float>(1, 1);    msg.t12 = P.at<float>(1, 2);    msg.t13 = P.at<float>(1, 3);
    msg.t20 = P.at<float>(2, 0);    msg.t21 = P.at<float>(2, 1);    msg.t22 = P.at<float>(2, 2);    msg.t23 = P.at<float>(2, 3);
    return msg;
}
