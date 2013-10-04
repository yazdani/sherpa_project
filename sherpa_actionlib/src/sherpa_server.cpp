#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sherpa_actionlib/trajAction.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <string>
#include <iostream>
#include <tf/LinearMath/Quaternion.h>

using namespace std;
class trajAction
{
protected:

  ros::NodeHandle nh_;
  ros::NodeHandle nh2_; 
  actionlib::SimpleActionServer<sherpa_actionlib::trajAction> as_; 
  std::string action_name_;
  sherpa_actionlib::trajFeedback feedback_;
  sherpa_actionlib::trajResult result_;
  ros::ServiceClient gms_c; 
  gazebo_msgs::GetModelState getmodelstate; 
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
  
  void executeCB(const sherpa_actionlib::trajGoalConstPtr &goal)
  {
    ROS_INFO("Action client registered, starting executing!");
    gms_c = nh2_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    
    getmodelstate.request.model_name="Sherpa_Human";
    // helper variables
    ros::Rate r(1);
 
    bool success = true;
    geometry_msgs::Twist tw;
    geometry_msgs::Pose ps;
    float vectordistance = 1.0;
    ROS_INFO("%s: Executing!", action_name_.c_str());
      
    ps.position.x = getmodelstate.response.pose.position.x;
    ps.position.y = getmodelstate.response.pose.position.y;     
    ps.position.z = getmodelstate.response.pose.position.z;
    ps.orientation.x = getmodelstate.response.pose.orientation.x;
    ps.orientation.y = getmodelstate.response.pose.orientation.y;
    ps.orientation.z = getmodelstate.response.pose.orientation.z;
   
    tw.linear.x = getmodelstate.response.twist.linear.x;
    tw.linear.y = getmodelstate.response.twist.linear.y;     
    tw.linear.z = getmodelstate.response.twist.linear.z;
    tw.angular.x = getmodelstate.response.twist.angular.x;
    tw.angular.y = getmodelstate.response.twist.angular.y;
    tw.angular.z = getmodelstate.response.twist.angular.z;

    
    float pythagoras = sqrt((
			   ((goal->setPoseGoalBusyGenius.position.x - ps.position.x)*(goal->setPoseGoalBusyGenius.position.x - ps.position.x))
			   + ((goal->setPoseGoalBusyGenius.position.y - ps.position.y)*(goal->setPoseGoalBusyGenius.position.y - ps.position.y)) 
			   +  ((goal->setPoseGoalBusyGenius.position.z - ps.position.z)*(goal->setPoseGoalBusyGenius.position.z - ps.position.z))));

    while(pythagoras > vectordistance)
      {
	if(as_.isPreemptRequested() || !ros::ok())
	  {
	    ROS_INFO("%s: Preempted", action_name_.c_str());
	    // set the action state to preempted
	    as_.setPreempted();
	    success = false;
	    break;
	  }
	gms_c.call(getmodelstate);
	
	ps.position.x = getmodelstate.response.pose.position.x;
	ps.position.y = getmodelstate.response.pose.position.y;     
	ps.position.z = getmodelstate.response.pose.position.z;
	ps.orientation.x = getmodelstate.response.pose.orientation.x;
	ps.orientation.y = getmodelstate.response.pose.orientation.y;
	ps.orientation.z = getmodelstate.response.pose.orientation.z;
	
	tw.linear.x = getmodelstate.response.twist.linear.x;
	tw.linear.y = getmodelstate.response.twist.linear.y;     
	tw.linear.z = getmodelstate.response.twist.linear.z;
	tw.angular.x = getmodelstate.response.twist.angular.x;
	tw.angular.y = getmodelstate.response.twist.angular.y;
	tw.angular.z = getmodelstate.response.twist.angular.z;
        ROS_INFO("%f pythagoras", pythagoras);
	  pythagoras = sqrt((
			     ((goal->setPoseGoalBusyGenius.position.x - ps.position.x)*(goal->setPoseGoalBusyGenius.position.x - ps.position.x))
			     + ((goal->setPoseGoalBusyGenius.position.y - ps.position.y)*(goal->setPoseGoalBusyGenius.position.y - ps.position.y)) 
			     +  ((goal->setPoseGoalBusyGenius.position.z - ps.position.z)*(goal->setPoseGoalBusyGenius.position.z - ps.position.z))));
	  
	  feedback_.getPoseOfBusyGenius = ps;
	  feedback_.getTwistOfBusyGenius= tw;
	  as_.publishFeedback(feedback_);
	  r.sleep();
      }
    // publish the feedback
    
    // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
 
    
    if(success)
      {
	result_.end = "Executing is finished";//feedback_.sequence[1];
        result_.getTwist = tw;
        result_.getPose = ps;
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
