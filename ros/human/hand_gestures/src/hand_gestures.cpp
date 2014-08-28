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

// TODO corriger decalage due au fait que image rgb et depth on une petite translation.

using namespace std;
using namespace cv;
using namespace pcl;
namespace enc = sensor_msgs::image_encodings;

HandSeg seg;
HaarDetect fist_detector;
HaarDetect palm_detector;

Mat K (3, 3, CV_64F);
Mat mask;
bool display = false;
int gest = 0;

Point2f getRightHand ();

void cloud_callback(const sensor_msgs::PointCloud2& msg) {
	PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);
	fromROSMsg (msg, *cloud);

	unsigned int h = cloud->height;
	unsigned int w = cloud->width;
	Mat depth (h, w, CV_32FC3);
	Mat image (h, w, CV_8UC3);
	for (size_t y=0; y<h; ++y) {
		for (size_t x=0; x<w; ++x) {
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

    int start = cv::getTickCount();
    Rect fist = fist_detector.detect(image, display);
    Rect palm = palm_detector.detect(image, display);
    int end = cv::getTickCount();
    float time_period = 1 / cv::getTickFrequency();

    Point2f right_hand = getRightHand();

    if (fist.area() && !palm.area())
        gest = 1; // fist only
    else if (!fist.area() && palm.area())
        gest = 2; // palm only
    else
        gest = 0; // other

    int thickness = -1;
    int lineType = 8;
    circle( image, right_hand, 5, Scalar( 0, 0, 255 ), thickness, lineType );
    rectangle (image, Point(fist.x, fist.y), Point(fist.x+fist.height, fist.y+fist.width), Scalar(255, 0, 0));
    rectangle (image, Point(palm.x, palm.y), Point(palm.x+palm.height, palm.y+palm.width), Scalar(0, 255, 0));
    imshow("Debug", image); waitKey(1);    
    ROS_INFO("Procesing time: %f s.", (end - start) * time_period);
}
/*
void cloud_callback(const sensor_msgs::Image& msg) {
    
    mask;
}
*/
// rosrun hand_gestures hand_gestures /camera/depth_registered/points gest
int main (int argc, char** argv) {
	assert (argc == 3 && "Usage : hand_gesture in_registered_cloud_topic out_gesture_topic");
	ros::init(argc, argv, "hand_gesture");
	ros::NodeHandle n;
	ros::Subscriber cloud_subscriber = n.subscribe(argv[1], 1, cloud_callback);
	//ros::Subscriber cloud_subscriber = n.subscribe(argv[1], 1, user_callback);
	ros::Publisher gest_publisher = n.advertise<std_msgs::Int32>(argv[2], 1);

    fist_detector.loadCascade("/home/gmanfred/devel/datasets/cascades/hands/fist/fist_best.xml");
    palm_detector.loadCascade("/home/gmanfred/devel/datasets/cascades/hands/palm/palm.xml");

    FileStorage fs("/home/gmanfred/.ros/camera_info/my_xtion.yml", FileStorage::READ);
    fs["camera_matrix"] >> K;

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
    double x = -transform.getOrigin().y();
    double y = transform.getOrigin().z();
    double z = transform.getOrigin().x();
    tf::Matrix3x3 R(transform.getRotation());
    Mat P = (Mat_<double>(4,4) << R[0][0], R[0][1], R[0][2], x,
                                 R[1][0], R[1][1], R[1][2], y, 
                                 R[2][0], R[2][1], R[2][2], z,
                                 0, 0, 0, 1);
    return P;    
}

Mat getTransform (string camera, string frame) {
    tf::TransformListener listener;

	tf::StampedTransform transform;
    try {
        listener.waitForTransform(camera, frame, ros::Time(0), ros::Duration(2.0) );
        listener.lookupTransform(camera, frame, ros::Time(0), transform);
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
    cout << t << endl;
    Point2d pt;
    pt.x = (K.at<double>(0,0) * t.at<double>(0) / t.at<double>(2)) + K.at<double>(0,2);
    pt.y = (K.at<double>(1,1) * t.at<double>(1) / t.at<double>(2)) + K.at<double>(1,2);
    cout << pt << endl;
    return pt;
}

Point2f getRightHand () {
    Mat T = getTransform("/openni_depth_frame", "right_hand_1");
    //Mat T = getTransform("/camera_rgb_optical_frame", "right_hand_1");
    return projectFrame2Image(T);
}
