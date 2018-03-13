#ifndef __SIDEWARDACTIONCLASS_H_INCLUDED__
#define __SIDEWARDACTIONCLASS_H_INCLUDED__

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_common/sidewardAction.h>
#include "PID.h"

class ActionClass
{
  private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<motion_common::sidewardAction> sideward_server_;
    std::string action_name_;
    motion_common::sidewardFeedback feedback_;
    motion_common::sidewardResult result_;
    ros::Publisher motor1_pub, motor4_pub;
    PID pid_;

    std_msgs::Int32 pwm;
    float pos_curr;
    float pos_prev;

  public:
    explicit ActionClass(std::string);
    ~ActionClass(void);
    void preemptCB(void);
    void executeCB(motion_common::sidewardGoalConstPtr);
    void set_pid(float, float, float, float, float, float);
};

#endif
