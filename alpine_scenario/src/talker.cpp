#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetPhysicsProperties.h>
#include <sstream>

void talker_1(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  
  ros::NodeHandle n;
  ros::NodeHandle m;
  ros::ServiceClient gmscl, gmscl2 = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  
  ros::ServiceClient smsl = m.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::GetModelState getmodelstate, getmodelstate2;
  gazebo_msgs::SetModelState setmodelstate;
  
  geometry_msgs::Pose start_pose;
  gazebo_msgs::ModelState modelstate;
  geometry_msgs::Twist start_twist;
  
  //  ros::Publisher posePub = m.advertise<gazebo_msgs::ModelState>("topic_name", 10);
  //std_msgs::Float64 msg;
  modelstate.model_name = (std::string) "quadrotor1";
  //modelstate.reference_frame = (std::string) "Alp_Landscape";
  getmodelstate.request.model_name ="quadrotor1";
  gmscl.call(getmodelstate);
  
  getmodelstate2.request.model_name ="actor1";
  //gmscl2.call(getmodelstate2);
 
  while(ros::ok()) 
  {  
    //update of the poses
  gmscl2.call(getmodelstate2);
  if(getmodelstate2.response.pose.position.x  < 16 && getmodelstate2.response.pose.position.x  > 0 && getmodelstate2.response.pose.position.y > -5 ) 
    {
      // ROS_INFO("HALLO TRUE");
      start_pose.position.x = getmodelstate2.response.pose.position.x + 4;
      start_pose.position.y = getmodelstate2.response.pose.position.y - 2;
      start_pose.position.z = getmodelstate2.response.pose.position.z + 3;
      start_pose.orientation.x = getmodelstate.response.pose.orientation.x;
      start_pose.orientation.y = getmodelstate.response.pose.orientation.y ;
      start_pose.orientation.z = getmodelstate.response.pose.orientation.z - 3;
      start_pose.orientation.w = getmodelstate.response.pose.orientation.w;
      /*
	start_twist.linear.x = getmodelstate.response.twist.linear.x;
      start_twist.linear.y = getmodelstate.response.twist.linear.y;
      start_twist.linear.z = getmodelstate.response.twist.linear.z;
      start_twist.angular.x = getmodelstate.response.twist.angular.x;
      start_twist.angular.y = getmodelstate.response.twist.angular.y;
      start_twist.angular.z = getmodelstate.response.twist.angular.z;
      */
    
    }  
    
    
    if(getmodelstate2.response.pose.position.y > -16 && getmodelstate2.response.pose.position.x  > 16)
      {
	//	ROS_INFO("NEXT ONE");
	//	start_pose.position.x = start_pose.position.x + 0.5;
	start_pose.position.y = getmodelstate2.response.pose.position.y - 2;
	start_pose.position.z = getmodelstate2.response.pose.position.z + 3;
	start_pose.orientation.x = getmodelstate.response.pose.orientation.x;
	start_pose.orientation.y = getmodelstate.response.pose.orientation.y ;
	start_pose.orientation.z = getmodelstate.response.pose.orientation.z - 3;
	start_pose.orientation.w = getmodelstate.response.pose.orientation.w; 
	
      }

if(getmodelstate2.response.pose.position.x  < 22 && getmodelstate2.response.pose.position.y <-16 && getmodelstate2.response.pose.position.x > -5 ) 
    {
      // ROS_INFO("HALLO TRUE 2");
       start_pose.position.x = getmodelstate2.response.pose.position.x - 2;
       //      start_pose.position.y = getmodelstate2.response.pose.position.y;
       start_pose.position.z = getmodelstate2.response.pose.position.z + 3;
      start_pose.orientation.x = getmodelstate.response.pose.orientation.x;
      start_pose.orientation.y = getmodelstate.response.pose.orientation.y ;
      start_pose.orientation.z = getmodelstate.response.pose.orientation.z - 3;
      start_pose.orientation.w = getmodelstate.response.pose.orientation.w;
      
    } 
      
    if(getmodelstate2.response.pose.position.x  < 0 && getmodelstate2.response.pose.position.y  > -19 && getmodelstate2.response.pose.position.x > -10) 
    {
      //  ROS_INFO("HALLO TRUE 3");
       start_pose.position.x = getmodelstate2.response.pose.position.x + 4;
       start_pose.position.y = getmodelstate2.response.pose.position.y ;
         start_pose.position.z = getmodelstate2.response.pose.position.z + 3;
      start_pose.orientation.x = getmodelstate.response.pose.orientation.x;
      start_pose.orientation.y = getmodelstate.response.pose.orientation.y ;
      start_pose.orientation.z = getmodelstate.response.pose.orientation.z - 3;
      start_pose.orientation.w = getmodelstate.response.pose.orientation.w;
      
    } 
												   /*
    if(getmodelstate2.response.pose.position.x  < 22 && getmodelstate2.response.pose.position.x  > -6 && getmodelstate2.response.pose.position.y < 0) 
    {
      ROS_INFO("HALLO TRUE 3");
       start_pose.position.x = getmodelstate2.response.pose.position.x - 2;
       //tart_pose.position.y = getmodelstate2.response.pose.position.y;
       start_pose.position.z = getmodelstate2.response.pose.position.z + 3;
      start_pose.orientation.x = getmodelstate.response.pose.orientation.x;
      start_pose.orientation.y = getmodelstate.response.pose.orientation.y ;
      start_pose.orientation.z = getmodelstate.response.pose.orientation.z - 3;
      start_pose.orientation.w = getmodelstate.response.pose.orientation.w;
      
    } 
*/ 
  modelstate.pose = start_pose;
  modelstate.twist = start_twist;

  setmodelstate.request.model_state = modelstate;
  smsl.call(setmodelstate);

  }  

}
