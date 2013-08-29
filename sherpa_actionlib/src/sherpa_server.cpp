#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sherpa_actionlib/trajAction.h>


class trajAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<sherpa_actionlib::trajAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  sherpa_actionlib::trajFeedback feedback_;
  sherpa_actionlib::trajResult result_;
  
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
  
    // helper variables
     ros::Rate r(1);
     bool success = true;

     // publish info to the console for the user
     ROS_INFO("%s: Executing!", action_name_.c_str());
     feedback_.sequence.clear();
     feedback_.sequence.push_back("Not ready! Is still working...");
     //    feedback_.sequence.push_back("Executing is finished!");
    
     //start executing the action
      
    for(int i=0; i<goal->start.size(); i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.end = "Executing is finished";//feedback_.sequence[1];
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
      }
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory");
  trajAction trajectory(ros::this_node::getName());
  ros::spin();
  return 0;
}
