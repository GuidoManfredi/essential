#include <tf/transform_listener.h>

#include "../pr2_commander/Arm.h"
#include "../pr2_commander/Gripper.h"
#include "../pr2_commander/Head.h"

class Robot {
  public:
    Robot();

    void grab ();
    void give ();
    void moveWrist (double step);
    
    void lookAt (std::string xyz_frame, double x, double y, double z);
    void lookAtHand (std::string camera, std::string hand, double offset);
    
    void scan (double start, double step, double end);
    void fullScan (double step);
    
    //joint_state (std::string joint_name);
    //joint_states (std::vector<std::string> joint_names);

  private:
    Arm* r_arm_;
    Arm* l_arm_;
    Gripper r_gripper_;
    Head head_;
    
    std::string main_camera_;
    tf::TransformListener tf_;
    
    std::vector<double> give_pose;
    std::vector<double> grab_pose;
    std::vector<double> lookat_pose;
    
    double wrist_pose_;
};
