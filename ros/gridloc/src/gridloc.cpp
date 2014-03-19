#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>

#include "../gridloc/Gridloc.h"
#include "../gridloc/pipeline2D.h"

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;

Mat image;
Pipeline2D p;

void images_cb (const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
	image = cv_ptr->image;

	vector<KeyPoint> circles;
	Mat gray;
	p.color2gray (image, gray);
    p.detectCircles (gray, circles);
    cout << "Number of circles: " << circles.size() << endl;
    p.drawCircles (image, circles);
    imshow ("Circles", image);
    waitKey(1);
}

// ./bin/gridloc /camera/rgb/image_rect_color
int main (int argc, char** argv) {
    assert (argc == 2 && "Usage : gridloc in_image_topic");

    ros::init(argc, argv, "gridloc");
    ros::NodeHandle n;

    ros::Subscriber sub_images = n.subscribe(argv[1], 1, images_cb);
    //ros::Publisher pub_target = n.advertise<>("/detection_target", 10);

  ros::Rate loop_rate(100);
    while (ros::ok()) {

        ros::spinOnce ();
        loop_rate.sleep();
    }

	return 0;
}
