#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetPhysicsProperties.h>

void chatterCallback(const std_msgs::Float64::ConstPtr &msg)
{
  
  ROS_INFO("%.20f", msg->data);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");
  
  ros::NodeHandle n;
  // gazebo_msgs::GetModelState getmodelstate;
  ros::Subscriber sub = n.subscribe("topic_name", 1000, chatterCallback);
   
 ros::spin();
  
  return 0;
}
