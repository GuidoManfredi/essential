#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include "reco_color/HistogramRecognition.h"
#include "RecoHist.h"

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

ros::Publisher pub_hist;
ros::Publisher pub_candidates;
vector<string> candidates;
RecoHist reco;

// Callback for pointcloud subscriber
bool recognize (reco_color::HistogramRecognition::Request &req,
								reco_color::HistogramRecognition::Response &res) {
	cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(req.sticker, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    res.result = 1;
    return false;
  }
  vector<string> candidates = reco.recognize (cv_ptr->image, 30);
  res.names = candidates;
  res.result = 0;
  
  return true;
}

int main(int argc, char **argv) {
	if (argc != 2) {
		std::cout << "Usage: reco_hist train_file" << std::endl;
		return 1;
	}
  ros::init(argc, argv, "reco_color_node");
  ros::NodeHandle n;
  
	// Input
	ros::ServiceServer srv_hist = n.advertiseService("/recognition_hist", recognize);
  
 	ros::Rate loop_rate(10);
  while (ros::ok()) {
	  ros::spinOnce();
	  loop_rate.sleep();
  }

  return 0;
}
