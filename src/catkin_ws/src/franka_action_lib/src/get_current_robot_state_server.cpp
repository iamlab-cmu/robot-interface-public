#include <ros/ros.h>
#include "franka_action_lib/get_current_robot_state_server.h"

namespace franka_action_lib
{
  std::mutex GetCurrentRobotStateServer::robot_state_buffer_mutex_;
  boost::circular_buffer<franka_action_lib::RobotState> GetCurrentRobotStateServer::robot_state_buffer_{1};

  GetCurrentRobotStateServer::GetCurrentRobotStateServer(std::string name) :  nh_("~")
  {
    nh_.param("robot_state_topic_name_", robot_state_topic_name_, std::string("/robot_state_publisher_node/robot_state"));

    ros::Subscriber sub = nh_.subscribe(robot_state_topic_name_, 10, robot_state_sub_cb);
    ros::ServiceServer service = nh_.advertiseService("get_current_robot_state_server", get_current_robot_state);
    ROS_INFO("Get Current Robot State Server Started");
    ros::spin();
  }

  void GetCurrentRobotStateServer::robot_state_sub_cb(const franka_action_lib::RobotState& robot_state)
  {
    if (robot_state_buffer_mutex_.try_lock()) {
      robot_state_buffer_.push_back(robot_state);
      robot_state_buffer_mutex_.unlock();
    }
  }

  bool GetCurrentRobotStateServer::get_current_robot_state(GetCurrentRobotStateCmd::Request &req, GetCurrentRobotStateCmd::Response &res)
  {
    ROS_DEBUG("Get Current Robot State Server request received.");
    robot_state_buffer_mutex_.lock();
    res.robot_state = robot_state_buffer_.back();
    robot_state_buffer_mutex_.unlock();
    ROS_DEBUG("Get Current Robot State Servier request processed.");
    
    return true;
  }
}
