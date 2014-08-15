#include <ros/ros.h>
#include "../pr2_commander/Robot.h"
#include "pr2_commander/Grab.h"
#include "pr2_commander/Give.h"
#include "pr2_commander/MoveWrist.h"

Robot pr2;

bool grab (pr2_commander::Grab::Request &req, 
           pr2_commander::Grab::Response &res) {
    ROS_INFO("Going for a grab");
    pr2.grab();
    return true;
}

bool give (pr2_commander::Give::Request &req, 
           pr2_commander::Give::Response &res) {
    ROS_INFO("Giving object.");
    pr2.give();
    return true;
}

bool moveWrist (pr2_commander::MoveWrist::Request &req, 
                pr2_commander::MoveWrist::Response &res) {
    ROS_INFO("Turning wrist");
    pr2.moveWrist(req.angle);
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_gripper");



    return 0;
}

