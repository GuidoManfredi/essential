#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf/transform_broadcaster.h>

#include "opencv2/highgui/highgui.hpp"

#include "../pom/POD.h"

// !!! ATTENTION ! NE JAMAIS METTRE 2 FOIS LA MEME IMAGE DANS LE MODELE !!!
// si des descripteurs sont en double, lors du matching ils ne seront pas pris
// en compte par siftgpu, vas savoir pourquoi...

// TODO Faire la doc
//      Revert the mono object mode
//      Tester avec un seul broadcaster
//      Faire les match avec FLANN
//      Blinder POM et POD mais mettre du debug pour faire remonter les problems ou les printer.
//      Blinder detect_object avec un max de ROS_INFO/WARN/ERR
//      FINIR LOCAL2GLOBAL LES CAS 4 ET 5
//      Faire en sorte de pouvoir creer un model meme quand on a que des images de front/top/bottom (cf petit beurre)
//      Dans create model ne pas laisser de trailing / ou alors le nom du dossier ne sera pas bien lu. (a corriger)
//      Faire le multi object draw dans opencv_display_from_tf
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

POD detecter;
Object object;
vector<Mat> Ps;
vector<string> names;
string camera_frame;

void broadcastPoses (vector<Mat> Ps);
tf::Transform mat2msg (Mat P);
tf::Matrix3x3 cvR2tfR(cv::Mat cv);
cv::Mat rosK2cvK(double fx, double fy, double cx, double cy);

void intrinsic_callback(const sensor_msgs::CameraInfoConstPtr& msg) {
    if (!detecter.isIntrinsicSet()) {
        cv::Mat K = rosK2cvK(msg->K[0], msg->K[4], msg->K[2], msg->K[5]);
        //cout << K << endl;
        detecter.setIntrinsic(K);
        ROS_INFO("Loaded intrinsic parameters");
    }
}

void image_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    //imshow ("Input", cv_ptr->image);
    //waitKey(1);
    
    int start = cv::getTickCount();
    detecter.process (cv_ptr->image, Ps, names); // MAIN CALL
    int end = cv::getTickCount();
    float time_period = 1 / cv::getTickFrequency();
    //ROS_INFO("Procesing time: %f s.", (end - start) * time_period);
}

// ./bin/detect_object image purfruit
// rosrun opencv_publisher stream 0 webcam image
// rosrun opencv_display display_poses_from_tf image

// rosrun pom detect_object /narrow_stereo/left/image_rect /narrow_stereo/left/camera_info narrow_stereo_l_stereo_camera_optical_frame objects_list.txt
// rosrun pom detect_object /narrow_stereo/left/image_rect /narrow_stereo/left/camera_info narrow_stereo_l_stereo_camera_optical_frame object_path
// rosrun pom detect_object image camera_info camera /home/gmanfred/devel/datasets/my_objects/pom/models/lait.yaml
int main (int argc, char** argv) {
	assert (argc == 5 && "Usage : detect_object in_image_topic in_intrisic_topic camera_frame objects_list");
	ros::init(argc, argv, "pom");
	ros::NodeHandle n;
	ros::Subscriber image_subscriber = n.subscribe(argv[1], 1, image_callback);
	ros::Subscriber intrinsic_subscriber = n.subscribe(argv[2], 1, intrinsic_callback);
	camera_frame = argv[3];

    ROS_INFO("Loading objects...");
    //int num_objects = detecter.loadObjectsFromList(argv[4]); // Multi objects
    int num_objects = 1;
    detecter.loadObject(argv[4]); // Mono objects
    ROS_INFO("... objects loaded.");

    vector<tf::TransformBroadcaster> br;
    br.resize(num_objects);

    ros::Rate loop_rate(20);
	while (ros::ok()) {
        ros::spinOnce();
        
        for (size_t i = 0; i < Ps.size(); ++i) {
            //cout << Ps[i] << endl;
            tf::Transform transform = mat2msg(Ps[i]);
            br[i].sendTransform(tf::StampedTransform(transform, ros::Time::now(), camera_frame, names[i]));
        }

        loop_rate.sleep();
    }

    return 0;
}

tf::Transform mat2msg (Mat P) {
    Mat cvR = P(Rect(0,0,3,3));
    tf::Matrix3x3 R = cvR2tfR(cvR);
    //tf::Vector3 t(P.at<float>(0, 3), P.at<float>(1, 3), P.at<float>(2, 3));
    tf::Vector3 t(P.at<float>(0, 3)/1000, P.at<float>(1, 3)/1000, P.at<float>(2, 3)/1000);
    return tf::Transform(R, t);
}

tf::Matrix3x3 cvR2tfR(cv::Mat cv) {
    return tf::Matrix3x3(cv.at<float>(0,0), cv.at<float>(0,1), cv.at<float>(0,2),
                          cv.at<float>(1,0), cv.at<float>(1,1), cv.at<float>(1,2),
                          cv.at<float>(2,0), cv.at<float>(2,1), cv.at<float>(2,2));
}

cv::Mat rosK2cvK(double fx, double fy, double cx, double cy) {
    return (Mat_<double>(3,3) << fx,  0, cx,
                                  0, fy, cy,
                                  0,  0,  1);
}

