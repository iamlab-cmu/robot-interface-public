#include "franka_action_lib/sensor_data/sensor_subscriber_handler.h"

namespace franka_action_lib
{
  SensorSubscriberHandler::SensorSubscriberHandler(ros::NodeHandle& nh) : shared_memory_handler_() {
    ROS_INFO("created_sensor_subscriber_handler");
  }                                                               


  void SensorSubscriberHandler::dummyTimeCallback(const std_msgs::String::ConstPtr& msg)
  {
    ROS_INFO("got a message");
    ros::Time begin = ros::Time::now();
  }


}
