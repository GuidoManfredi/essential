#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "std_msgs/Int32.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "../Gestures/HandSeg.h"
#include "../Gestures/HaarDetect.h"

// TODO Recuperer uniquement image RGB, on n'utilise pas la profondeur pour l'instant
//      faire 2 callbacks differents et une seule fonction de traitement.

using namespace std;
using namespace cv;
using namespace pcl;
namespace enc = sensor_msgs::image_encodings;

tf::TransformListener* listener;

HandSeg seg;
HaarDetect fist_detector;
HaarDetect palm_detector;

Mat K (3, 3, CV_64F);
Mat mask;
bool display = false;
int gest = 0;

int hand_padding = 75;
float min_distance = 0.0;
float max_distance = 0.0;

Point2f getLeftHand ();
bool inside (Point2f pt, Rect rect);
Mat getTransform (string camera, string frame);

int getGest (Mat image) {
    int gest = 0;
    // Find position of hand in image
    Point2f left_hand = getLeftHand();
    // Gesture detectors
    Mat gray;
    cvtColor( image, gray, CV_BGR2GRAY );
    equalizeHist( gray, gray );
    Rect fist = fist_detector.detect(gray, left_hand, hand_padding, display);
    Rect palm = palm_detector.detect(gray, left_hand, hand_padding, display);
    // Head distance detection
    Mat camera_head = getTransform ("/camera_rgb_optical_frame", "/head_1");
    float camera_head_distance = camera_head.at<double>(0,3) * camera_head.at<double>(0,3)
                               + camera_head.at<double>(1,3) * camera_head.at<double>(1,3)
                               + camera_head.at<double>(2,3) * camera_head.at<double>(2,3);
    cout << "Camera/head distance: " << camera_head_distance << endl;

    int thickness = 2;
    int lineType = 8;
    circle( image, left_hand, 5, Scalar( 0, 0, 255 ), -1, lineType );
    rectangle (image, Point(left_hand.x + fist.x - hand_padding, left_hand.y + fist.y - hand_padding),
                      Point(left_hand.x + fist.x + fist.height - hand_padding, left_hand.y + fist.y + fist.width - hand_padding),
                      Scalar(255, 0, 0), thickness, lineType);
    rectangle (image, Point(left_hand.x + palm.x - hand_padding, left_hand.y + palm.y - hand_padding),
                      Point(left_hand.x + palm.x + palm.height - hand_padding, left_hand.y + palm.y + palm.width - hand_padding),
                      Scalar(0, 255, 0), thickness, lineType);
    imshow("Debug", image); waitKey(1);

    if (fist.area() && !palm.area())
        gest = 1; // fist only
    else if (!fist.area() && palm.area())
        gest = 2; // palm only
    else if (camera_head_distance < min_distance
            || camera_head_distance > max_distance)
        gest = 3;
    else
        gest = 0; // other
    return gest;
}

void cloud2rgbd(PointCloud<PointXYZRGB>::Ptr cloud, Mat &image, Mat &depth) {
	for (size_t y=0; y<cloud->height; ++y) {
		for (size_t x=0; x<cloud->width; ++x) {
			PointXYZRGB pt;
			pt = cloud->at(x,y);
			depth.at<Vec3f>(y, x)[0] = pt.x;
			depth.at<Vec3f>(y, x)[1] = pt.y;
			depth.at<Vec3f>(y, x)[2] = pt.z;
			int rgb = *reinterpret_cast<int*>(&pt.rgb);
            image.at<Vec3b>(y, x)[0] = (rgb & 0xff); // B
            image.at<Vec3b>(y, x)[1] = ((rgb >> 8) & 0xff); // G
            image.at<Vec3b>(y, x)[2] = ((rgb >> 16) & 0xff); // R
		}
	}
}

void cloud_callback(const sensor_msgs::PointCloud2& msg) {
	PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);
	fromROSMsg (msg, *cloud);
	
	unsigned int h = cloud->height;
	unsigned int w = cloud->width;
	Mat image (h, w, CV_8UC3);
    Mat depth (h, w, CV_32FC3);
	cloud2rgbd(cloud, image, depth);
	
	int start = cv::getTickCount();
	int gest = getGest(image);
	int end = cv::getTickCount();
    cout << "Gest: " << gest << endl;
    float time_period = 1 / cv::getTickFrequency();
    ROS_INFO("Procesing time: %f s.", (end - start) * time_period);
}
/*
void cloud_callback(const sensor_msgs::Image& msg) {
    
    mask;
}
*/
// rosrun hand_gestures hand_gestures /camera/depth_registered/points gest 0.7 1.2
int main (int argc, char** argv) {
	assert (argc == 5 && "Usage : hand_gesture in_registered_cloud_topic out_gesture_topic min_distance max_distance");
	ros::init(argc, argv, "hand_gesture");
	ros::NodeHandle n;
	ros::Subscriber cloud_subscriber = n.subscribe(argv[1], 1, cloud_callback);
	//ros::Subscriber cloud_subscriber = n.subscribe(argv[1], 1, user_callback);
	ros::Publisher gest_publisher = n.advertise<std_msgs::Int32>(argv[2], 1);

    listener = new tf::TransformListener();

    fist_detector.loadCascade("/home/gmanfred/devel/datasets/cascades/hands/fist/fist_best.xml");
    palm_detector.loadCascade("/home/gmanfred/devel/datasets/cascades/hands/palm/palm.xml");

    FileStorage fs("/home/gmanfred/.ros/camera_info/my_xtion.yml", FileStorage::READ);
    fs["camera_matrix"] >> K;

    min_distance = atof(argv[3]);
    max_distance = atof(argv[4]);

    ros::Rate rate(30);
	while (ros::ok()) {
        ros::spinOnce();
        
        std_msgs::Int32 msg;
        msg.data = gest;
        gest_publisher.publish(msg);
        
        rate.sleep();
    }

    return 0;
}

Mat transform2mat (tf::StampedTransform transform) {
    // Transform to camera optical frame
    /*
    double x = -transform.getOrigin().y();
    double y = transform.getOrigin().z();
    double z = transform.getOrigin().x();
    */
    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    double z = transform.getOrigin().z();
    tf::Matrix3x3 R(transform.getRotation());
    Mat P = (Mat_<double>(4,4) << R[0][0], R[0][1], R[0][2], x,
                                 R[1][0], R[1][1], R[1][2], y, 
                                 R[2][0], R[2][1], R[2][2], z,
                                 0, 0, 0, 1);
    return P;    
}

Mat getTransform (string camera, string frame) {
	tf::StampedTransform transform;
    try {
        listener->waitForTransform(camera, frame, ros::Time(0), ros::Duration(2.0) );
        listener->lookupTransform(camera, frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
    
    return transform2mat (transform);
}

Point2f projectFrame2Image (Mat T) {
    Mat t = (Mat_<double>(3, 1) << T.at<double>(0, 3),
                                  T.at<double>(1, 3),
                                  T.at<double>(2, 3));
    Point2d pt;
    pt.x = (K.at<double>(0,0) * t.at<double>(0) / t.at<double>(2)) + K.at<double>(0,2);
    pt.y = (K.at<double>(1,1) * t.at<double>(1) / t.at<double>(2)) + K.at<double>(1,2);
    return pt;
}

Point2f getLeftHand () {
    Mat T = getTransform("/camera_rgb_optical_frame", "left_hand_1");
    //Mat T = getTransform("/camera_rgb_optical_frame", "right_hand_1");
    return projectFrame2Image(T);
}

bool inside (Point2f pt, Rect rect) {
    return (pt.x > rect.x && pt.y > rect.y && pt.x < rect.x + rect.width && pt.y < rect.y + rect.height);
}
