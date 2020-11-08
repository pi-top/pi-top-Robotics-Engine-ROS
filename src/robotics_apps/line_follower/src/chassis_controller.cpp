#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"


void callback(const std_msgs::Float64::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "chassis_controller");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/line_follower/control_effort", 1, callback);
  //ros::Publisher sub = n.advertise("/cmd_vel", 1000, chatterCallback);
  ros::spin();

  return 0;
}