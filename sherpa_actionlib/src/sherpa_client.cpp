#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sherpa_actionlib/trajAction.h>
//#include "gazebo_msgs/GetModelState.h"
//#include "gazebo_msgs/SetModelState.h"


int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_action");

 // create the action client
  //ros::NodeHandle nh_;
  //ros::ServiceClient gms_c;
  //gazebo_msgs::GetModelState getmodelstate; 
  //gazebo_msgs::ModelState modelstate;
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<sherpa_actionlib::trajAction> ac("trajectory", true);
 
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  sherpa_actionlib::trajGoal goal;
  geometry_msgs::Twist tw;
  geometry_msgs::Pose ps;
  // gms_c = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  //  getmodelstate.request.model_name="Sherpa_Human";

  ps.position.x = 8;
  ps.position.y = 4;     
  ps.position.z = 0.5;
  ps.orientation.x = 0.0;
  ps.orientation.y = 0.0;
  ps.orientation.z = 0.0;

  tw.linear.x = 0;
  tw.linear.y = 0;     
  tw.linear.z = 0;
  tw.angular.x = 0;
  tw.angular.y = 0;
  tw.angular.z = 0;
  goal.setTwistGoalBusyGenius = tw;
  goal.setPoseGoalBusyGenius = ps;
 
  //wait for the action to return
  ac.sendGoal(goal);
 
  
  
  bool finished_before_timeout = ac.waitForResult(ros::Duration(100.0));
 
    if(finished_before_timeout)
    { 
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
      ROS_INFO("Action did not finish before the time out.");
 
  return 0;
}
