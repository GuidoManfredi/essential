#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <tf/transform_broadcaster.h>

#include "opencv2/highgui/highgui.hpp"

#include "../pom/POD.h"

// !!! ATTENTION ! NE JAMAIS METTRE 2 FOIS LA MEME IMAGE DANS LE MODELE !!!
// si des descripteurs sont en double, lors du matching ils ne seront pas pris
// en compte par siftgpu, vas savoir pourquoi...

// ./bin/detect_object image purfruit
// rosrun opencv_publisher stream 0 webcam image
// rosrun opencv_display display_poses_from_tf image
// TODO lire les parametres intrinseques depuis topic
//      pouvoir recuperer noms des objets dont on a les poses
//      Recuperer soft pour faire les corners (qui est dans le package)
//      Donner un dossier out pour les objet.yaml
//      Faire la doc
//      Charger models en lisant liste (+fonctions de debug quand fichier non disponible
//      Blinder POM et POD mais mettre du debug pour faire remonter les problems ou les printer.
//      Blinder create_objetct et detect_object avec un max de ROS_INFO/WARN/ERR
//      Faire le multi object draw dans opencv_display_from_tf
//      Donner des noms au objets a l'interieur du fichier yaml ? (ou nom du fichier suffit ?)
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

POD detecter;
Object object;
vector<Mat> Ps;
vector<string> names;

void broadcastPoses (vector<Mat> Ps);
tf::Transform mat2msg (Mat P);
tf::Matrix3x3 cvR2tfR(cv::Mat cv);

void image_callback(const sensor_msgs::ImageConstPtr& msg) {
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
    detecter.process (cv_ptr->image, Ps);
    int end = cv::getTickCount();
    float time_period = 1 / cv::getTickFrequency();
    ROS_INFO("Procesing time: %f s.", (end - start) * time_period);
}

// ./bin/detect_object images purfruit
// rosrun opencv_display display_poses /camera/rgb/image_rect_color pose
int main (int argc, char** argv) {
	assert (argc == 3 && "Usage : detect_objects in_image_topic object_name");
	ros::init(argc, argv, "pom");
	ros::NodeHandle n;
	ros::Subscriber cloud_subscriber = n.subscribe(argv[1], 1, image_callback);

    detecter.loadIntrinsic ("/home/gmanfred/.ros/camera_info/webcam_gilgamesh_opencv.yml");
    detecter.loadObject(argv[2]);
	while (ros::ok()) {
        ros::spinOnce();
        broadcastPoses(Ps);
    }

    return 0;
}

void broadcastPoses (vector<Mat> Ps) {
    static tf::TransformBroadcaster br;
    for (size_t i = 0; i < Ps.size(); ++i) {
        tf::Transform transform = mat2msg(Ps[i]);
        string object_name ("pipo");
        //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", object_name));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), object_name, "world"));
    }
}
  

tf::Transform mat2msg (Mat P) {
    //P = P.inv();
    Mat cvR = P(Rect(0,0,3,3));
    tf::Matrix3x3 R = cvR2tfR(cvR);
    tf::Vector3 t(P.at<float>(0, 3), P.at<float>(1, 3), P.at<float>(2, 3));
    return tf::Transform(R, t);
}

tf::Matrix3x3 cvR2tfR(cv::Mat cv) {
    return tf::Matrix3x3(cv.at<float>(0,0), cv.at<float>(0,1), cv.at<float>(0,2),
                          cv.at<float>(1,0), cv.at<float>(1,1), cv.at<float>(1,2),
                          cv.at<float>(2,0), cv.at<float>(2,1), cv.at<float>(2,2));
}
