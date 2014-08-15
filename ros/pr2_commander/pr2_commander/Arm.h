#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class Arm {
  public:
    Arm(bool left_arm);
    ~Arm();

    void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal);
    
    pr2_controllers_msgs::JointTrajectoryGoal rollWrist (double angle);
    pr2_controllers_msgs::JointTrajectoryGoal moveWholeArmTo (std::vector<double> positions);

    pr2_controllers_msgs::JointTrajectoryGoal moveArmTo(std::string joint_name,
                                                        double position);
    pr2_controllers_msgs::JointTrajectoryGoal moveArmTo(std::vector<std::string> joint_names,
                                                        std::vector<double> positions);

    actionlib::SimpleClientGoalState getState();

  private:
    TrajClient* traj_client_;
};

