#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperFindContactAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperSlipServoAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperEventDetectorAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperSlipServoAction> SlipClient;
// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperFindContactAction> ContactClient;
// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperEventDetectorAction> EventDetectorClient;
// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper{
  public:
    Gripper();
    ~Gripper();
    // high level functions
    void grab();
    void give();
    
  private:
    // open
    void open();
    // close gently until both fingers touch something
    void findTwoContacts();
    // grab firmly even if object is slipping
    void slipServo();
    // detect an event
    void detect();
    
    GripperClient* gripper_client_;  
    ContactClient* contact_client_;
    SlipClient* slip_client_;
    EventDetectorClient* event_detector_client_;
};
