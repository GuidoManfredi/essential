// http://opencv.willowgarage.com/wiki/DisplayManyImages

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/opencv.hpp"

#include "seg_plans_objs/ImageArray.h"

namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;


void show_many_images(char* window_name, vector<Mat> images, Size big_size) {
	// Create a new 3 channel image
  Mat big_image = Mat::zeros( big_size, CV_8UC3);
  // Maximum number of images in a row/column
	int max_img_row = 5, max_img_col = 5;
	// Space between two images/to the frame, in pixels
	int padding = 20;
	int N = images.size();
	if (N > max_img_row*max_img_col)
		cout << "Warning : showManyImages : images vector size > 25, showing only the 25 first images." << endl;

	float size = big_size.height/max_img_col;
	float scale = 1.0;
	int max = 0;
	for (int i=0, m=padding, n=padding; i<N; ++i, m+= padding + size) {
		if( i % max_img_row == 0 && m!= padding) {
    	m = padding;
    	n+= padding + size;
    }
    max = (images[i].cols > images[i].rows)? images[i].cols: images[i].rows;
    scale = (float) ( (float) max/size );
		Rect roi = Rect(m, n, (int) images[i].cols/scale, (int) images[i].rows/scale);
		//cout << roi << endl;
		Mat ROI = big_image(roi);
		resize(images[i], ROI, roi.size());
	}
	
	imshow (window_name, big_image);
}

void image_callback(const seg_plans_objs::ImageArrayConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	vector<Mat> images;
	for (int i=0; i<msg->array.size(); ++i) {
		try {
		  cv_ptr = cv_bridge::toCvCopy(msg->array[i], enc::BGR8);
		}
		catch (cv_bridge::Exception& e) {
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}
		images.push_back (cv_ptr->image);
	}
	if (images.size() > 0)
		show_many_images("Clusters", images, Size(640, 480));
}

int main (int argc, char** argv) {
	if (argc != 2) {
		cout << "Usage : display_poses vec_image_topic" << endl;
		return 1;
	}

	ros::init(argc, argv, "opencv_display_poses");
	ros::NodeHandle n;

	// Subscribe to relevant information
	ros::Subscriber image_subscriber = n.subscribe(argv[1], 10, image_callback);
  
  // test
  /*
  Mat img1 = imread("/home/gmanfred/Pictures/fusee1.png");
  Mat img2 = imread("/home/gmanfred/Pictures/kangos.jpg");
  Mat img3 = imread("/home/gmanfred/Pictures/main.png");
  Mat img4 = imread("/home/gmanfred/Pictures/sign.jpg");
  vector<Mat> images;
  images.push_back(img1);
  images.push_back(img2);
  images.push_back(img3);
  images.push_back(img4);
  images.push_back(img4);
  images.push_back(img4);
  images.push_back(img2);
  images.push_back(img4);
  images.push_back(img4);
  images.push_back(img2);
  images.push_back(img4);
  images.push_back(img2);
  images.push_back(img2);
  images.push_back(img2);
  show_many_images ("kikoulol", images, Size(640, 480));
	waitKey(0);
	*/	  
	ros::Rate loop_rate(10);
	while (ros::ok()) {
	  ros::spinOnce();
	  waitKey(1);
	  loop_rate.sleep();
	}
  return 0;  
}

