#include "../pr2_commander/Arm.h"

using namespace std;

Arm::Arm(bool left_arm)
{
    if (left_arm) {
        traj_client_ = new TrajClient("l_arm_controller/joint_trajectory_action", true);
    } else {
        // tell the action client that we want to spin a thread by default
        traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
    }
    
    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
}

Arm::~Arm()
{
    delete traj_client_;
}

//! Sends the command to start a given trajectory
void Arm::startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal) {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
}

pr2_controllers_msgs::JointTrajectoryGoal Arm::rollWrist (double angle) {
    return moveArmTo("r_wrist_roll_joint", angle);
}

pr2_controllers_msgs::JointTrajectoryGoal Arm::moveWholeArmTo (vector<double> positions) {
    assert(positions.size() == 7 && "moveWholeArmTo : Not enough positions provided");
    
    vector<string> joint_names;
    joint_names.push_back("r_shoulder_pan_joint");
    joint_names.push_back("r_shoulder_lift_joint");
    joint_names.push_back("r_upper_arm_roll_joint");
    joint_names.push_back("r_elbow_flex_joint");
    joint_names.push_back("r_forearm_roll_joint");
    joint_names.push_back("r_wrist_flex_joint");
    joint_names.push_back("r_wrist_roll_joint");
    return moveArmTo(joint_names, positions);
}

pr2_controllers_msgs::JointTrajectoryGoal Arm::moveArmTo(string joint_name,
                                                         double position) {
    vector<string> joint_names;
    joint_names.push_back(joint_name);
    vector<double> positions;
    positions.push_back(position);
    return moveArmTo(joint_names, positions);
}

pr2_controllers_msgs::JointTrajectoryGoal Arm::moveArmTo(std::vector<std::string> joint_names,
                                                         std::vector<double> positions) {
    pr2_controllers_msgs::JointTrajectoryGoal goal;
    
    size_t n = joint_names.size();
    goal.trajectory.joint_names.resize(n);
    goal.trajectory.points.resize(1); // only one point
    goal.trajectory.points[0].positions.resize(n);
    goal.trajectory.points[0].velocities.resize(n);

    for (size_t i = 0; i < n; ++i) {
        goal.trajectory.joint_names[i] = joint_names[i];
        goal.trajectory.points[0].positions[i] = positions[i];
        goal.trajectory.points[0].velocities[i] = 0.0;
    }
    goal.trajectory.points[0].time_from_start = ros::Duration(1.0);
    return goal;
}

actionlib::SimpleClientGoalState Arm::getState()
{
    return traj_client_->getState();
}


