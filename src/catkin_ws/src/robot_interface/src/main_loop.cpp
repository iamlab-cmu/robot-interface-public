#include <ros/ros.h>
#include <signal.h>

#include <actionlib/server/simple_action_server.h>
#include <robot_interface/execute_skillAction.h>
#include <robot_interfae/run_loop.h>

bool RunLoop::init(ros::NodeHandle& nh): node_handle(nh) {
}

void RunLoop::starting(const ros::Time&) {
}

void RunLoop::update(const ros::Time& /* time */,
                     const ros::Duration& period) {
}

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

// TODO: This main loop should be inside some main runner class.
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_interface_main_node");
  ros::NodeHandle nh;

  // Read parameters from param server.
  //
  // Instantiate new skills (Note that this should happen in the loop)
  // TODO: How do we instantiate new skills on the fly?
  execute_skillAction exec_skill_action("skill")


  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  // signal(SIGINT, mySigintHandler);

  int count = 0;
  ros::Rate loop_rate(1000);
  while (ros::ok()) {
    ros::Time last_time = ros::Time::now();


    if (count % 100 == 0) {
      // Publish feedback message
      count = 0
    }
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  
  return 0;
}
