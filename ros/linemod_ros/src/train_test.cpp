#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include "../Linemod/OnlineGrabber.h"

// TODO
//  Check load/save
//  Multiobjects

using namespace std;
using namespace pcl;

OnlineGrabber og(10.0, 0.95);
visualization::PCLVisualizer viewer("Training");
float detection_thresh = 0.95;
float detection_score = 0.0;

void cloud_cb(const PointCloudType::ConstPtr& msg) {
	if (!viewer.updatePointCloud (msg, "single_cloud"))
		viewer.addPointCloud (msg, "single_cloud");
    // Si reconnaissance sous seuil

    PointXYZ min, max;
    if (detection_score < detection_thresh) {
        og.grab(msg);
        detection_score = og.detect(msg, min, max);
    } else {
        detection_score = og.detect(msg, min, max);
    }
    cout << "Detection: " << detection_score << endl;

    // Show result
    viewer.removeShape ("cube", 0);
    viewer.addCube (min.x, max.x, min.y, max.y, min.z, max.z);
}

// Se mettre dans un dossier vide, ou seront ecris les templates.
// rosrun linemod_ros train_test /camera/depth_registered/points model.lmt
int main(int argc, char** argv) {
    assert(argc == 3 && "Usage: train_test cloud_in model_out");

	ros::init(argc, argv, "linemod_online_train");
	ros::NodeHandle n;
	ros::Subscriber sub_clouds = n.subscribe(argv[1], 1, cloud_cb);

	viewer.setBackgroundColor (0, 0, 0);
	viewer.initCameraParameters ();

	ros::Rate loop_rate(10);
	while (ros::ok() && !viewer.wasStopped()) {
	    viewer.spinOnce(1);
		ros::spinOnce();
		loop_rate.sleep ();
	}

    og.save(argv[2]);

	return 0;
}
