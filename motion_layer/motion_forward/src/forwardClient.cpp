#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_common/forwardAction.h>

#include <dynamic_reconfigure/server.h>
#include <motion_forward/clientConfig.h>

motion_common::forwardGoal goal;
// create pointer since client can't be initialised before ros::init()
actionlib::SimpleActionClient<motion_common::forwardAction> *client;

void feedback_cb(motion_common::forwardActionFeedback msg)
{
  ROS_INFO("Remaining distance: %f", msg.feedback.remaining);
}

void callback_client(motion_forward::clientConfig &config, uint32_t level)
{
  //set goal
  if(config.send)
    goal.goal = config.goal;
    goal.checks = config.checks;
    ROS_INFO("Sending goal:%f checks:%d", config.goal, config.checks);
    client->sendGoal(goal);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "client");
  ros::NodeHandle n;
  // here initilise the client pointer
  actionlib::SimpleActionClient<motion_common::forwardAction> temp_client("forward", true);
  client = &temp_client;

  ros::Subscriber feedback_sub = n.subscribe<motion_common::forwardActionFeedback>("forward/feedback", 1000, feedback_cb);

  dynamic_reconfigure::Server<motion_forward::clientConfig> dynamic_server;
  dynamic_reconfigure::Server<motion_forward::clientConfig>::CallbackType cb;

  cb = boost::bind(&callback_client, _1, _2);
  dynamic_server.setCallback(cb);

  ROS_INFO("Waiting for server to start. ");
  client->waitForServer();
  ROS_INFO("Action server started, sending goal");

  ros::spin();
  return 0;
}
