#include <std_msgs/Int32.h>

#include <string>

#include "../include/sidewardActionClass.h"
#include "../include/PID.h"

ActionClass::ActionClass(std::string name):
sideward_server_(nh_, name, boost::bind(&ActionClass::executeCB, this, _1), false), action_name_(name), pid_(0, 0, 0, 0, 0, 0)
{
  sideward_server_.registerPreemptCallback(boost::bind(&ActionClass::preemptCB, this));
  // sideward motion depends only on motor1 and motor4
  motor1_pub = nh_.advertise<std_msgs::Int32>("/pwm/motor1", 1000);
  motor4_pub = nh_.advertise<std_msgs::Int32>("/pwm/motor4", 1000);
  pos_curr = 0;
  pos_prev = 0;
  sideward_server_.start();
}

ActionClass::~ActionClass(void){}

void ActionClass::executeCB(motion_common::sidewardGoalConstPtr goal_)
{
  int frequency = 10;
  ros::Rate loop(frequency);

  if(!sideward_server_.isActive())
    return;

  pos_curr = goal_->goal;
  bool reached = false;
  int count = 0;

  while(!sideward_server_.isPreemptRequested() && ros::ok() && count < goal_->checks)
  {
    double output = pid_.compute(pos_curr, 0);
    pwm.data = pid_.toPWM(output);

    if(pwm.data <= pid_.get_band() && pwm.data >= -pid_.get_band())
    {
      reached = true;
      pwm.data = 0;
      motor1_pub.publish(pwm);
      motor4_pub.publish(pwm);
      sideward_server_.setPreempted();
      count++;
    }
    else
    {
      reached = false;
      count = 0;
    }

    if(sideward_server_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      sideward_server_.setPreempted();
      reached = false;
      break;
    }

    feedback_.remaining = pid_.get_prev_error();
    sideward_server_.publishFeedback(feedback_);
    motor1_pub.publish(pwm);
    motor4_pub.publish(pwm);
    ROS_INFO("%s send %d pwm to arduino sideward", ros::this_node::getName().c_str(), pwm.data);

    ros::spinOnce();
    loop.sleep();
  }



}

void ActionClass::preemptCB(void)
{
  pwm.data = 0;
  motor1_pub.publish(pwm);
  motor4_pub.publish(pwm);
  ROS_INFO("%s: Preempted", action_name_.c_str());
  sideward_server_.setPreempted();
}

void ActionClass::set_pid(float min_val, float max_val, float band, float kp, float ki, float kd)
{
  pid_.updateConstants(min_val, max_val, band, kp, ki, kd);
}
