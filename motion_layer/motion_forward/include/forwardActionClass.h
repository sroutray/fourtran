#ifndef __FORWARDACTIONCLASS_H_INCLUDED__
#define __FORWARDACTIONCLASS_H_INCLUDED__

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_common/forwardAction.h>
#include "PID.h"

class ActionClass
{
  private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<motion_common::forwardAction> forward_server_;
    std::string action_name_;
    motion_common::forwardFeedback feedback_;
    motion_common::forwardResult result_;
    ros::Publisher motor2_pub, motor3_pub;
    PID pid_;

    std_msgs::Int32 pwm;
    float pos_curr;
    float pos_prev;

  public:
    explicit ActionClass(std::string);
    ~ActionClass(void);
    void preemptCB(void);
    void executeCB(motion_common::forwardGoalConstPtr);
    void set_pid(float, float, float, float, float, float);
};

#endif
