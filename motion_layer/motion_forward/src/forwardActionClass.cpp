#include <std_msgs/Int32.h>

#include <string>

#include "../include/forwardActionClass.h"
#include "../include/PID.h"

ActionClass::ActionClass(std::string name):
forward_server_(nh_, name, boost::bind(&ActionClass::executeCB, this, _1), false), action_name_(name), pid_(0, 0, 0, 0, 0, 0)
{
  forward_server_.registerPreemptCallback(boost::bind(&ActionClass::preemptCB, this));
  // forward motion depends only on motor2 and motor3
  motor2_pub = nh_.advertise<std_msgs::Int32>("/pwm/motor2", 1000);
  motor3_pub = nh_.advertise<std_msgs::Int32>("/pwm/motor3", 1000);
  pos_curr = 0;
  pos_prev = 0;
  forward_server_.start();
}

ActionClass::~ActionClass(void){}

void ActionClass::executeCB(motion_common::forwardGoalConstPtr goal_)
{
  int frequency = 10;
  ros::Rate loop(frequency);

  if(!forward_server_.isActive())
    return;

  pos_curr = goal_->goal;
  bool reached = false;
  int count = 0;

  while(!forward_server_.isPreemptRequested() && ros::ok() && count < goal_->checks)
  {
    double output = pid_.compute(pos_curr, 0);
    pwm.data = pid_.toPWM(output);

    if(pwm.data <= pid_.get_band() && pwm.data >= -pid_.get_band())
    {
      reached = true;
      pwm.data = 0;
      motor2_pub.publish(pwm);
      motor3_pub.publish(pwm);
      forward_server_.setPreempted();
      count++;
    }
    else
    {
      reached = false;
      count = 0;
    }

    if(forward_server_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      forward_server_.setPreempted();
      reached = false;
      break;
    }

    feedback_.remaining = pid_.get_prev_error();
    forward_server_.publishFeedback(feedback_);
    motor2_pub.publish(pwm);
    motor3_pub.publish(pwm);
    ROS_INFO("%s send %d pwm to arduino forward", ros::this_node::getName().c_str(), pwm.data);

    ros::spinOnce();
    loop.sleep();
  }



}

void ActionClass::preemptCB(void)
{
  pwm.data = 0;
  motor2_pub.publish(pwm);
  motor3_pub.publish(pwm);
  ROS_INFO("%s: Preempted", action_name_.c_str());
  forward_server_.setPreempted();
}

void ActionClass::set_pid(float min_val, float max_val, float band, float kp, float ki, float kd)
{
  pid_.updateConstants(min_val, max_val, band, kp, ki, kd);
}
