#include "franka_action_lib/sensor_data/sensor_subscriber_handler.h"

namespace franka_action_lib
{
  SensorSubscriberHandler::SensorSubscriberHandler(ros::NodeHandle& nh)
  : shared_memory_handler_  ()
  , nh_                     (nh) {
    ROS_INFO("created_sensor_subscriber_handler");
  }


    void SensorSubscriberHandler::dummyTimeCallback(const std_msgs::Float64::ConstPtr& f_data)
  {
    ROS_INFO("got a message");

    shared_memory_handler_.loadSensorData_dummy_Unprotected(f_data, 0);
  }


}
