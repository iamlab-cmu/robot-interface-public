#include <ros/ros.h>
#include <signal.h>

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "main_loop", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  // signal(SIGINT, mySigintHandler);

  int count = 0;
  ros::Rate loop_rate(1000);
  while (ros::ok()) {

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
