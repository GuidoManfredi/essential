#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "hand_msgs/Rectangle.h"
#include "hand_detect/HandDetection.h"
#include "hand_track/HandTrack.h"
#include "Detector.h"

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;

string cascade_file = "/home/gmanfred/devel/ros/Vision_pipeline/icaro/hand_detect/cascades/poing.xml";
Detector det (cascade_file);
int mode = 2;
hand_msgs::Rectangle target;

hand_msgs::Rectangle detect (Mat img) {
	vector<Rect> hands = det.detect (img);
	hand_msgs::Rectangle rect;
	for (size_t i=0; i<hands.size (); ++i) {
		rect.x = hands[i].x;
		rect.y = hands[i].y;
		rect.width = hands[i].width;
		rect.height = hands[i].height;
	}
	return rect;
}

void images_cb (const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

	if ( mode == 2) {
		cout << "detecting non stop ..." << endl;
		target = detect (cv_ptr->image);
	}
	else if ( mode == 1) {
		cout << "detecting ..." << endl;
		target = detect (cv_ptr->image);
		if (target.width > 0 && target.height > 0)
			mode = 0;
	}
}

bool detect_hands (hand_detect::HandDetection::Request &req,
									hand_detect::HandDetection::Response &res) {
	mode = req.mode;
	res.mode = mode;
	return true;
}

int main (int argc, char** argv) {
	if (argc != 2) {
		cout << "Usage : hand_detect images_topic" << endl;
		return 1;
	}

  ros::init(argc, argv, "hand_detect");
  ros::NodeHandle n;

	ros::Subscriber sub_images = n.subscribe(argv[1], 10, images_cb);
	ros::Publisher pub_target = n.advertise<hand_msgs::Rectangle>("/detection_target", 10);
  ros::ServiceServer srv_hands = n.advertiseService("/detect_hands", detect_hands);
	ros::ServiceClient srv_client_track = n.serviceClient<hand_track::HandTrack>("track_hands");
  
  ros::Rate loop_rate(10);
  while (ros::ok()) {
  	if (target.width > 0 && target.height > 0) {
  		cout << "Detection ! Calling track_hands service" << endl;
  		hand_track::HandTrack srv;
  		srv.request.rect.push_back(target);
  		if (!srv_client_track.call(srv))
				ROS_ERROR("Failed to call service track_hands");
			pub_target.publish (target);
			// reinit target
			target.width = 0;
			target.height = 0;
  	}
	  ros::spinOnce ();
	  loop_rate.sleep();
	}

	return 0;
}
////////////////////////////////////////////////////////////////////////////////
// TESTING CODE
////////////////////////////////////////////////////////////////////////////////
/*
void test ();
void draw (Mat img, vector<Rect> hands);

const static Scalar colors[] =  { CV_RGB(0,0,255),
																	CV_RGB(0,128,255),
																	CV_RGB(0,255,255),
																	CV_RGB(0,255,0),
																	CV_RGB(255,128,0),
																	CV_RGB(255,255,0),
																	CV_RGB(255,0,0),
																	CV_RGB(255,0,255)} ;

void test () {
	Detector det ("/home/gmanfred/devel/ros/Vision_pipeline/icaro/hand_detect/cascades/poing.xml");
	
  VideoCapture cap(1); // open the default camera
  if(!cap.isOpened())  // check if we succeeded
      return;

  Mat edges;
  namedWindow("hand",1);
  for(;;) {
      Mat frame;
      cap >> frame; // get a new frame from camera
      vector<Rect> hands;
	    hands = det.detect (frame);
	    draw (frame, hands);
      imshow("hand", frame);
      if(waitKey(30) >= 0) break;
  }
}

void draw (Mat img, vector<Rect> hands) {
	Scalar color;
	for (size_t i=0; i<hands.size(); ++i) {
    Scalar color = colors[i%8];
    rectangle (img, hands[i], color, 3, 8, 0 );
  }
}
*/
