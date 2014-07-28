#include <iostream>

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	if(argc != 4)
	{
		cout << "Usage : stream cameraDeviceNum(910 for asus xtion) camera_name topic_name" << endl;
		return -1;
	}

	ros::init(argc, argv, "stream");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::CameraPublisher pub(it.advertiseCamera(argv[3], 1));

	cv_bridge::CvImage cv_img;
	sensor_msgs::Image ros_img;
	
	camera_info_manager::CameraInfoManager info_mgr(ros::NodeHandle(argv[2]), argv[2]);
	sensor_msgs::CameraInfo info = info_mgr.getCameraInfo();

	VideoCapture capture;
	//capture.open(CV_CAP_OPENNI); // capture from asus xtion
	//capture.open(CV_CAP_OPENNI_ASUS); // capture from asus xtion
	capture.open(atoi(argv[1])); // capture from video device #argv[1]
	//capture.set(CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ);
	//capture.set(CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_SXGA_30HZ);
	//capture.set(CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_QVGA_30HZ);
	if (!capture.isOpened())
	  cout << "capture device " << argv[1] << " failed to open!" << endl;
	
	Mat in;
	ros::Rate loop_rate(10);
	while(ros::ok()) {
		capture >> in;
		cv_img.header.stamp = ros::Time::now();
		cv_img.header.frame_id = argv[2];
		cv_img.encoding = "bgr8";
		cv_img.image = in;
		
        info.header.stamp = cv_img.header.stamp;
        info.header.frame_id = cv_img.header.frame_id;
        info.width = in.cols;
        info.height = in.rows;

		cv_img.toImageMsg(ros_img);
		pub.publish(ros_img, info);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

