#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sherpa_actionlib/trajAction.h>
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/SetModelState.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <string>
#include <iostream>
using namespace std;
class trajAction
{
protected:

  ros::NodeHandle nh_;
  ros::NodeHandle nh2_; 
 // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<sherpa_actionlib::trajAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  sherpa_actionlib::trajFeedback feedback_;
  sherpa_actionlib::trajResult result_;
  ros::ServiceClient gms_c; 
  gazebo_msgs::GetModelState getmodelstate; 
  float db;
  gazebo_msgs::SetModelState setmodelstate;
  gazebo_msgs::ModelState modelstate;

public:
  
  trajAction(std::string name) :
    as_(nh_, name, boost::bind(&trajAction::executeCB, this, _1), false),
  action_name_(name)
  {

    as_.start();
    ROS_INFO("Waiting for action client to start...");
  }
  
  ~trajAction(void)
  {
  }
  
  string inverter(float val)
  {
   ostringstream Str;
    db = val;
    Str << db;
    string digit(Str.str());
     return digit;
  }
 
  void executeCB(const sherpa_actionlib::trajGoalConstPtr &goal)
  {
    ROS_INFO("Action client registered, starting executing!");
    gms_c = nh2_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    
    getmodelstate.request.model_name="Sherpa_Human";
    // helper variables
    ros::Rate r(1);
    bool success = true;
    
    gms_c.call(getmodelstate);
    // publish info to the console for the user
    ROS_INFO("%s: Executing!", action_name_.c_str());
    //feedback_.feed ="Not ready! Is still working...";
    
    if(feedback_.pose.size()==0)
      {
    feedback_.pose.push_back("pose: ");
    feedback_.pose.push_back(" position: ");
    feedback_.pose.push_back("  x: " + inverter(getmodelstate.response.pose.position.x));
    feedback_.pose.push_back("  y: " + inverter(getmodelstate.response.pose.position.y));     
    feedback_.pose.push_back("  z: "+ inverter(getmodelstate.response.pose.position.z));     
    feedback_.pose.push_back(" orientation");
    feedback_.pose.push_back("  x: "+ inverter(getmodelstate.response.pose.orientation.x));     
    feedback_.pose.push_back("  y: "+inverter(getmodelstate.response.pose.orientation.y));     
    feedback_.pose.push_back("  z: "+inverter(getmodelstate.response.pose.orientation.z));
    feedback_.pose.push_back("twist: ");
    feedback_.pose.push_back(" linear: ");
    feedback_.pose.push_back("  x: "+inverter(getmodelstate.response.twist.linear.x));     
    feedback_.pose.push_back("  y: "+inverter(getmodelstate.response.twist.linear.y));     
    feedback_.pose.push_back("  z: "+inverter(getmodelstate.response.twist.linear.z));
    feedback_.pose.push_back(" angular: ");
    feedback_.pose.push_back("  x: "+inverter(getmodelstate.response.twist.angular.x));
    feedback_.pose.push_back("  y: "+inverter(getmodelstate.response.twist.angular.y));
    feedback_.pose.push_back("  z: "+inverter(getmodelstate.response.twist.angular.z));
      }
    //feedback_.feed ="Executing is finished!";
    if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
      }
    // publish the feedback
    as_.publishFeedback(feedback_);
    // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
    r.sleep();
    
    
    if(success)
      {
	result_.res = "Executing is finished";//feedback_.sequence[1];
      }	
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    // set the action state to succeeded
    as_.setSucceeded(result_);     
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory");
  trajAction trajectory(ros::this_node::getName());
  ros::spin();
  return 0;
}
