#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "../include/PID.h"
#include "../include/sidewardActionClass.h"

#include <dynamic_reconfigure/server.h>
#include <motion_sideward/pidConfig.h>

ActionClass *object;

void callback_pid(motion_sideward::pidConfig &config, uint32_t level)
{
  ROS_INFO("Set Kp:%f Ki:%f Kd:%f Max_val:%f Min_val:%f Band:%f", config.kp, config.ki, config.kd, config.max_val, config.min_val, config.band);
  object->set_pid(config.min_val, config.max_val, config.band, config.kp, config.ki, config.kd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sideward");
  ros::NodeHandle nh;


  ROS_INFO("%s is waiting for goal: ", ros::this_node::getName().c_str());
  object = new ActionClass(ros::this_node::getName());

  dynamic_reconfigure::Server<motion_sideward::pidConfig> dynamic_server;
  dynamic_reconfigure::Server<motion_sideward::pidConfig>::CallbackType cb;

  cb = boost::bind(&callback_pid, _1, _2);
  dynamic_server.setCallback(cb);

  ros::spin();
  return 0;
}
