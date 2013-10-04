
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
ros::Time last_recieved_joy_message_time_;
//using gazebo;
class TeleopMoving
{
  public:
   TeleopMoving();
  private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh1_;
  // SherpaHumanPlugin shp;
  int linear_, angular_;
  double l_scale_, a_scale_;
  double run_button;
  ros::Publisher vel_pub_, vel_pub1_;
  ros::Subscriber joy_sub_;
 
  ros::Time last_recieved_joy_message_time_;
 
   void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
};

  TeleopMoving::TeleopMoving()
  {  
 ROS_INFO("Starting the Keyboard:");
  std::cout << "Pushing the Buttons\n" << std::endl;
  std::cout << "Y : Pointing out\n" << std::endl;
 std::cout << "X : Left Rotation\n" << std::endl;
 std::cout << "B : Right Rotation\n" << std::endl;
 std::cout << "LB & RB : Going backwards \n" << std::endl;
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
      vel_pub1_ = nh1_.advertise<std_msgs::String>("cmd_pose", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopMoving::joyCallback, this);
  }

  //~TeleopMoving() {}

  void TeleopMoving::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
 
  last_recieved_joy_message_time_ = ros::Time::now();
  geometry_msgs::Twist twister;
  std_msgs::String poser;
 //pressing the two sticks

 bool l1_stick = ((0 < joy->axes.size()) &&  joy->axes[0]); //LEFT R&L
 bool l2_stick = ((0 < joy->axes.size()) &&  joy->axes[1]); //LEFT UP AND DOWN

 bool r1_stick = ((0 < joy->axes.size()) &&  joy->axes[3]); //RIGHT R&L
 bool r2_stick = ((0 < joy->axes.size()) &&  joy->axes[4]); //RIGHT UP AND DOWN

   /* 
 bool axes_up = ((2 < joy->buttons.size()) && joy->axes[2]);  
  bool axes_down = ((3 < joy->buttons.size()) && joy->axes[3]);
  */

 // ROS_INFO("twister-dirty %d", twister.dirty);
 //pressing of the several buttons

  bool button_A = ((0 < joy->buttons.size()) && joy->buttons[0]);
  bool button_B = ((1 < joy->buttons.size()) && joy->buttons[1]);
  bool button_X = ((2 < joy->buttons.size()) && joy->buttons[2]);
  bool button_Y = ((3 < joy->buttons.size()) && joy->buttons[3]);
  bool button_LB = ((4 < joy->buttons.size()) && joy->buttons[4]);
  bool button_RB = ((5 < joy->buttons.size()) && joy->buttons[5]);
 


  if(l1_stick == true && button_LB == false && button_Y == false)
    { //go ahead
    twister.linear.x = 1; 
    poser.data="walk"; 
  }else if(l1_stick == true && button_LB == true && button_Y == false)
    { //go back
    twister.linear.x = -1;
    poser.data="walk"; 
  }else  if(r1_stick == true && button_RB == false && button_Y == false)
    { //go left
    twister.linear.y = -1;
    poser.data="walk"; 
 }else if(r1_stick == true && button_RB == true && button_Y == false)
    { //go right
    twister.linear.y = 1;
    poser.data="walk"; 
   }else if(button_X == true)
    { //rotate left
      twister.angular.z = 1;
    }else if(button_B == true)
    { //rotate right
      twister.angular.z = -1;
    }else if(button_Y == true )
    {
      poser.data="point";
    }
 
 
 
  
 vel_pub_.publish(twister); 
  vel_pub1_.publish(poser);

 //vel_pub1_.publish(str);
}

int main(int argcs, char** argv)
{
  ros::init(argcs, argv, "teleop_sherpa_human");
   TeleopMoving teleop_moving;
  ros::spin();
}
