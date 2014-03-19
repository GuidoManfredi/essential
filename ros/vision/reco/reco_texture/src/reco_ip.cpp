#include "ros/ros.h"

#include "reco_texture/TexturedRecognition.h"

#include "IPRecognition.h"

using namespace std;
// ip = interest points
//IPRecognition reco;

// Callback for pointcloud subscriber
bool recognize (reco_texture::TexturedRecognition::Request &req,
								reco_texture::TexturedRecognition::Response &res) {
	
	//reco.recognize (req.names);
	
	return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "reco_texture_node");
  ros::NodeHandle n;

  ros::ServiceServer srv_plans = n.advertiseService("/recognition_texture", recognize);
  
  cout << "Recognition with texture service ready." << endl;
 	ros::Rate loop_rate(10);
  while (ros::ok()) {
	  ros::spinOnce();
	  loop_rate.sleep();
  }

  return 0;
}
