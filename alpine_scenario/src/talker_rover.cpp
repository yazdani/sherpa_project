#include "ros/ros.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/GetPhysicsProperties.h"

int main(int argc, char **argv)
{
ros::init(argc, argv, "talker_rover");
  
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
  modelstate.model_name = (std::string) "rover_base";
  //modelstate.reference_frame = (std::string) "Alp_Landscape";
  getmodelstate.request.model_name ="rover_base";
  gmscl.call(getmodelstate);
  
  getmodelstate2.request.model_name ="actor1";
  //gmscl2.call(getmodelstate2);

   while(ros::ok()) 
  {
      gmscl2.call(getmodelstate2);
   
      if(getmodelstate2.response.pose.position.x > 20.97 && getmodelstate2.response.pose.position.y > -20  && getmodelstate2.response.pose.position.z < -2)
	{
     start_pose.position.x = getmodelstate2.response.pose.position.x;
      start_pose.position.y = getmodelstate2.response.pose.position.y+5;
      start_pose.position.z = getmodelstate2.response.pose.position.z -1 ;
       
       start_pose.orientation.x = 0;
       start_pose.orientation.y = 0;
       start_pose.orientation.z = 30;
       start_pose.orientation.w = -30;
      // start_pose.angular.x =2;
  }
      
      if(getmodelstate2.response.pose.position.x < 19.65  && getmodelstate2.response.pose.position.x > -20 && getmodelstate2.response.pose.position.y > -20  && getmodelstate2.response.pose.position.z < -2)
        {
	   start_pose.position.x = getmodelstate2.response.pose.position.x + 4 ;
	  start_pose.position.y = getmodelstate2.response.pose.position.y;
	  start_pose.position.z = getmodelstate2.response.pose.position.z -0.5 ;

        start_pose.orientation.x = 0;
        start_pose.orientation.y = 0;
        start_pose.orientation.z = 0;
        start_pose.orientation.w = 1;

	}
      
  modelstate.pose = start_pose;
  modelstate.twist = start_twist;

  setmodelstate.request.model_state = modelstate;
  smsl.call(setmodelstate);
  }

   return 0;
}
