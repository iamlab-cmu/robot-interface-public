#include <ros/ros.h>
#include "franka_action_lib/get_current_robolib_status_server.h"

namespace franka_action_lib
{
  std::mutex GetCurrentRobolibStatusServer::robolib_status_buffer_mutex_;
  boost::circular_buffer<franka_action_lib::RobolibStatus> GetCurrentRobolibStatusServer::robolib_status_buffer_{1};

  GetCurrentRobolibStatusServer::GetCurrentRobolibStatusServer(std::string name) :  nh_("~")
  {
    nh_.param("robolib_status_topic_name", robolib_status_topic_name_, std::string("/robolib_status_publisher_node/robolib_status"));

    ros::Subscriber sub = nh_.subscribe(robolib_status_topic_name_, 10, robolib_status_sub_cb);
    ros::ServiceServer service = nh_.advertiseService("get_current_robolib_status_server", get_current_robolib_status);
    ROS_INFO("Get Current Robolib Status Server Started");
    ros::spin();
  }

  void GetCurrentRobolibStatusServer::robolib_status_sub_cb(const franka_action_lib::RobolibStatus& robolib_status)
  {
    if (robolib_status_buffer_mutex_.try_lock()) {
      robolib_status_buffer_.push_back(robolib_status);
      robolib_status_buffer_mutex_.unlock();
    }
  }

  bool GetCurrentRobolibStatusServer::get_current_robolib_status(GetCurrentRobolibStatusCmd::Request &req, GetCurrentRobolibStatusCmd::Response &res)
  {
    ROS_DEBUG("Get Current Robolib Status Server request received.");
    robolib_status_buffer_mutex_.lock();
    res.robolib_status = robolib_status_buffer_.back();
    robolib_status_buffer_mutex_.unlock();
    ROS_DEBUG("Get Current Robolib Status Servier request processed.");
    
    return true;
  }
}
