

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
class TeleopMoving
{
  public:
   TeleopMoving();
  private:
  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  double run_button;
  ros::Publisher vel_pub_, vel_pub1_;
  ros::Subscriber joy_sub_;
 
  ros::Time last_recieved_joy_message_time_;
 
   void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
};

  TeleopMoving::TeleopMoving():
    linear_(1),
    angular_(2)
  {  

  ROS_INFO("setting of the default");
  
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_linear", a_scale_, a_scale_);
  nh_.param("scale_angular", l_scale_, l_scale_);
 nh_.param("run_button", run_button, run_button);

  
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  //vel_pub1_ = nh_.advertise<std_msgs::String>("cmd_vel1", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopMoving::joyCallback, this);
  ROS_INFO("finish of the default");
}

  //~TeleopMoving() {}

  void TeleopMoving::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
   ROS_INFO("setting of the joycall");
  last_recieved_joy_message_time_ = ros::Time::now();
 geometry_msgs::Twist twister;
 // std_msgs::String str;
 //pressing the two sticks

 bool l1_stick = ((0 < joy->axes.size()) &&  joy->axes[0]); //LEFT R&L
 bool l2_stick = ((0 < joy->axes.size()) &&  joy->axes[1]); //LEFT UP AND DOWN

 bool r1_stick = ((0 < joy->axes.size()) &&  joy->axes[3]); //RIGHT R&L
 bool r2_stick = ((0 < joy->axes.size()) &&  joy->axes[4]); //RIGHT UP AND DOWN

   /* 
 bool axes_up = ((2 < joy->buttons.size()) && joy->axes[2]);  
  bool axes_down = ((3 < joy->buttons.size()) && joy->axes[3]);
  */
 ROS_INFO("\n");
 ROS_INFO("l1_stick: %d", l1_stick);
 ROS_INFO("l2_stick: %d", l2_stick);
 ROS_INFO("r1_stick: %d", r1_stick);
 ROS_INFO("r2_stick: %d", r2_stick);
 ROS_INFO("\n");
 ROS_INFO("twister-dirty %d", twister.dirty);
 //pressing of the several buttons
  bool button_A = ((0 < joy->buttons.size()) && joy->buttons[0]);
  bool button_B = ((1 < joy->buttons.size()) && joy->buttons[1]);
  bool button_X = ((2 < joy->buttons.size()) && joy->buttons[2]);
  bool button_Y = ((3 < joy->buttons.size()) && joy->buttons[3]);
  bool button_LB = ((4 < joy->buttons.size()) && joy->buttons[4]);
  bool button_RB = ((5 < joy->buttons.size()) && joy->buttons[5]);
 

  ROS_INFO("\n");
  ROS_INFO("button_A %d", button_A);
  ROS_INFO("button_B %d", button_B);
  ROS_INFO("button_X %d", button_X);
  ROS_INFO("button_Y %d", button_Y);
  ROS_INFO("button_LB %d", button_LB);
  ROS_INFO("button_RB %d", button_RB);  
  ROS_INFO("\n");

  if(l1_stick == true && button_LB == false)
    { //go ahead
    twister.linear.x = 1; 
    ROS_INFO("twister.x %f\n", twister.linear.x);
  }else if(l1_stick == true && button_LB == true)
    { //go back
    twister.linear.x = -1; 
    ROS_INFO("twister.x %f\n", twister.linear.x);
  }else  if(r1_stick == true && button_RB == false)
    { //go left
    twister.linear.y = -1; 
    ROS_INFO("twister.linear.y %f\n", twister.linear.y);
  }else if(r1_stick == true && button_RB == true)
    { //go right
    twister.linear.y = 1; 
    ROS_INFO("twister.linear.y %f\n", twister.linear.y);
    }else if(button_X == true)
    { //rotate left
      twister.angular.z = 1;
      ROS_INFO("twister.angular.z %f\n", twister.angular.z);
    }else if(button_B == true)
    { //rotate right
      twister.angular.z = -1;
       ROS_INFO("twister.angular.z %f\n", twister.angular.z);
    }
 
 
 
  
 ROS_INFO("setting of the joycall");
 ROS_INFO("twister.x %f", twister.linear.x);
 ROS_INFO("twister.y %f", twister.linear.y);
 ROS_INFO("twister.z %f", twister.linear.z);
 vel_pub_.publish(twister);
 //vel_pub1_.publish(str);
}

int main(int argcs, char** argv)
{
  ros::init(argcs, argv, "teleop_sherpa_human");
  ROS_INFO("HELLLOO");
   TeleopMoving teleop_moving;

   ROS_INFO("HI");
  ros::spin();
}
