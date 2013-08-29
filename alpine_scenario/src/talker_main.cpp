#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetPhysicsProperties.h>
#include <sstream>
//#include "talker_quadrotor2.cpp"
#include "talker.cpp"
//#include "talker_rover.cpp"

int main(int argc, char **argv)
{
  talker_1(argc,argv);
  // talker_2(argc,argv);
  //talker_rover(argc,argv);

return 0;
}
