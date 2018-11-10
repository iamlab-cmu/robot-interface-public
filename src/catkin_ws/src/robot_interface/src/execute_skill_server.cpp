#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robot_interface/execute_skillAction.h>

// include <actionlib_tutorials/FibonacciAction.h>

class execute_skillAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  actionlib::SimpleActionServer<robot_interface::execute_skillAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  robot_interface::execute_skillFeedback feedback_;
  robot_interface::execute_skillResult result_

  //actionlib_tutorials::FibonacciFeedback feedback_;
  //actionlib_tutorials::FibonacciResult result_;

public:

  execute_skillAction(std::string name) :
    as_(nh_, name, boost::bind(&execute_skillAction::executeCB, this, _1), false),
    action_name_(name) {
    as_.start();
  }

  ~execute_skillAction(void) { }

  void executeCB(const robot_interface::execute_skillGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1000);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating execute_skill sequence of order %i " \
        "with seeds %i, %i", action_name_.c_str(), goal->order,
        feedback_.sequence[0], feedback_.sequence[1]);

    // start executing the action
    for(int i=1; i<=goal->order; i++) {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(
          feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success) {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "execute_skill");

  execute_skillAction skill("execute_skill");
  ros::spin();

  return 0;
}
