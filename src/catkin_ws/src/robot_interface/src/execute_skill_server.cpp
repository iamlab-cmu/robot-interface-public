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

  // TODO: Shouldn't we pass in the node handle here?
  execute_skillAction(std::string name) :
    as_(nh_, name, boost::bind(&execute_skillAction::executeCB, this, _1), false),
    action_name_(name) {
    as_.start();
  }

  ~execute_skillAction(void) { }

  void executeCB(const robot_interface::execute_skillGoalConstPtr &goal)
  {
    // helper variables (Run the loop in 1KHZ)
    ros::Rate r(1);
    bool success = true;

    // Clear the feedback for this message
    feedback_.execution_result.clear();

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating execute_skill sequence of order %i " \
        "with seeds %i, %i", action_name_.c_str(), goal->topic,
        feedback_.sequence[0], feedback_.sequence[1]);

    // Start executing the action
    //
    // First parse the parameters
    float initial_value = goal->initial_sensor_values[0];
    
    // Now create the trajectory generator
    int traj_gen_type = goal->traj_gen_type;
    TrajectoryGenerator generator = TrajectoryGenerator(goal->traj_gen_params,
                                                        &nh_)
    

    // TODO: There should be some way to check if this new skill has finished 
    // executing or not.

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
