#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sherpa_actionlib/trajAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_action");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<sherpa_actionlib::trajAction> ac("trajectory", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  sherpa_actionlib::trajGoal goal;
  goal.start.push_back(0);
  goal.start.push_back(1);
  goal.start.push_back(2);
  goal.start.push_back(3);
  goal.start.push_back(4);
  goal.start.push_back(5);
  goal.start.push_back(6);
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
