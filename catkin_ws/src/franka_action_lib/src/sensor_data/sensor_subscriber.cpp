#include <ros/ros.h>
//#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include "franka_action_lib/sensor_data/sensor_subscriber_handler.h"



int main(int argc, char **argv)
{

  ros::NodeHandle n;

  franka_action_lib::SensorSubscriberHandler handler(n);
  ros::Subscriber sub = n.subscribe("dummy_time", 1000, &franka_action_lib::SensorSubscriberHandler::dummyTimeCallback, &handler);

  ros::spin();

  return 0;
}