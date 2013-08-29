#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>


//static const std::string
class MoveTrajectory{
   
private:
  ros::NodeHandle node;
  ros::ServiceClient client;
  ros::ServiceServer server;
  geometry_msgs::Twist twist;
  // SimpleActionClient *action_client;
  
 public:
  MoveTrajectory(){}
    //create a client function for the IK service
    
    //wait for the action server to come up
    //   while(ros::ok() && !action_client->waitForServer(ros::Duration(5.0))){
    //  ROS_INFO("Waiting for the joint_trajectory_action action server to come up");}

  //}

    ~MoveTrajectory(){
       // delete action_client;
   }
};


int main(int argc, char** argv){

  //init ROS node
  ros::init(argc, argv, "Moving the busy genius");
  
  MoveTrajectory move_traj_exec = MoveTrajectory();

  //ROS_INFO("Waiting for the busy genius");
  ros::spin();
  
  return 0;
}

