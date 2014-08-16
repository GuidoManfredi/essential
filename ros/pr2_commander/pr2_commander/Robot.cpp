#include "../pr2_commander/Robot.h"

using namespace std;

Robot::Robot() {
    l_arm_ = new Arm(true);
    r_arm_ = new Arm(false);
    
    double give[7] = {-0.1389710028810902, -0.31546987912598, 0.226784426550382, -0.471201872391641, -0.3410163620188146, -0.08481635605757976, 6.5737827590068845};
    give_pose.assign(&give[0], &give[0]+7);
    double grab[7] = {-0.27742505386725647, 0.00015377897674039714, 0.012390023693844121, -1.1478601773982497, -2.664965233170632, -1.201472790413669, 6.141869709178211};
    grab_pose.assign(&grab[0], &grab[0]+7);
    double lookat[7] = {0.13909779173423997, -0.08249546089631181, -0.002522975158555374, -1.1743181315681047, -0.8870928628321056, -0.0969553447653656, 6.678769783207206};
    lookat_pose.assign(&lookat[0], &lookat[0]+7);
    
    wrist_pose_ = 0.0;
}

void Robot::grab () {
    r_arm_->startTrajectoryBlock(r_arm_->moveWholeArmTo(grab_pose));
    r_gripper_.grab();
    r_arm_->startTrajectoryBlock(r_arm_->moveWholeArmTo(lookat_pose));
    lookAtTarget ("head_mount_kinect_rgb_optical_frame", "r_gripper_tool_frame");
}

void Robot::give () {
    r_arm_->startTrajectoryBlock(r_arm_->moveWholeArmTo(give_pose));
    r_gripper_.give();
}

void Robot::moveArm ( bool arm, double shoulder_pan, double shoulder_lift,
                        double upper_roll, double elbow_flex,
                        double forearm_roll,
                        double wrist_flex, double wrist_roll) {
    vector<double> positions;
    positions.push_back(shoulder_pan);
    positions.push_back(shoulder_lift);
    positions.push_back(upper_roll);
    positions.push_back(elbow_flex);
    positions.push_back(forearm_roll);
    positions.push_back(wrist_flex);
    positions.push_back(wrist_roll);
    
    if (arm == true)
        r_arm_->startTrajectory(l_arm_->moveWholeArmTo(positions));
    else
        r_arm_->startTrajectory(r_arm_->moveWholeArmTo(positions));
}

void Robot::moveWrist (double step) {
    if (wrist_pose_ <= 1e-4)
        r_arm_->rollWrist(0.0);      
    wrist_pose_ += step;
    r_arm_->startTrajectory(r_arm_->rollWrist(wrist_pose_));
}

void Robot::lookAt(string camera, string xyz_frame, double x, double y, double z) {
    head_.lookAt(camera, xyz_frame, x, y, z);
}

void Robot::lookAtTarget (string camera, string target) {
    tf::StampedTransform transform;
    try {
        tf_.lookupTransform(camera, target, ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    double z = transform.getOrigin().z();
    // camera frame look at xyz in camera frame
    lookAt (camera, camera, x, y, z);
}

void Robot::scan (double start, double step, double end) {

}

void Robot::fullScan (double step) {
    //r_arm_.startTrajectory(arm.moveWrist(step));
}
