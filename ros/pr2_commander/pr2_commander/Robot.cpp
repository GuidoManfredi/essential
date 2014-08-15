#include "../pr2_commander/Robot.h"

using namespace std;

Robot::Robot() {
    l_arm_ = new Arm(true);
    r_arm_ = new Arm(false);

    main_camera_ = "";
    
    double give[7] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    give_pose.assign(&give[0], &give[0]+7);
    double grab[7] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    give_pose.assign(&grab[0], &grab[0]+7);
    double lookat[7] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    lookat_pose.assign(&lookat[0], &lookat[0]+7);
    
    wrist_pose_ = 0.0;
}

void Robot::grab () {
    r_arm_->startTrajectory(r_arm_->moveWholeArmTo(grab_pose));
    r_gripper_.grab();
    r_arm_->startTrajectory(r_arm_->moveWholeArmTo(lookat_pose));
}

void Robot::give () {
    r_arm_->startTrajectory(r_arm_->moveWholeArmTo(give_pose));
    r_gripper_.give();
}

void Robot::moveWrist (double step) {
    if (wrist_pose_ <= 1e-4)
        r_arm_->rollWrist(0);
        
    wrist_pose_ += step;
    r_arm_->rollWrist(wrist_pose_);
    
    
}

void Robot::lookAt(string xyz_frame, double x, double y, double z) {
    head_.lookAt(main_camera_, xyz_frame, x, y, z);
}

//"/r_wrist_roll_link"
void Robot::lookAtHand (string camera, string hand, double offset) {
    tf::StampedTransform transform;
    try {
        tf_.lookupTransform(camera, hand, ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    double z = transform.getOrigin().z() + offset; // add offset along z
    // camera frame look at xyz in camera frame
    lookAt (camera, x, y, z);
}

void Robot::scan (double start, double step, double end) {

}

void Robot::fullScan (double step) {
    //r_arm_.startTrajectory(arm.moveWrist(step));
}
