#include <ros/ros.h>
#include "../pr2_commander/Robot.h"
#include "pr2_commander/Grab.h"
#include "pr2_commander/Give.h"
#include "pr2_commander/MoveWrist.h"
#include "pr2_commander/LookAt.h"

Robot* pr2;

bool grab (pr2_commander::Grab::Request &req, 
           pr2_commander::Grab::Response &res) {
    ROS_INFO("Going for a grab");
    pr2->grab();
    return true;
}

bool give (pr2_commander::Give::Request &req, 
           pr2_commander::Give::Response &res) {
    ROS_INFO("Giving object.");
    pr2->give();
    return true;
}

bool moveWrist (pr2_commander::MoveWrist::Request &req, 
                pr2_commander::MoveWrist::Response &res) {
    ROS_INFO("Turning wrist %f", req.angle);
    pr2->moveWrist(req.angle);
    return true;
}

bool lookAt (pr2_commander::LookAt::Request &req, 
                pr2_commander::LookAt::Response &res) {
    pr2->lookAtTarget(req.camera, req.target);
    return true;
}

// roslaunch pr2_gripper_sensor_action pr2_gripper_sensor_actions.launch 
// rosservice call /Grab false
// rosservice call /LookAt "head_mount_kinect_rgb_optical_frame" "/r_wrist_roll_link"
int main(int argc, char** argv) {
    ros::init(argc, argv, "grab_scan_give");
    ros::NodeHandle n;
    pr2 = new Robot();
    ros::ServiceServer grab_service = n.advertiseService("Grab", grab);
    ros::ServiceServer give_service = n.advertiseService("Give", give);
    ros::ServiceServer move_wrist_service = n.advertiseService("MoveWrist", moveWrist);
    ros::ServiceServer look_at_service = n.advertiseService("LookAt", lookAt);
    ROS_INFO("Grab/Scan/Give ready.");
    
    ros::spin();

    return 0;
}

