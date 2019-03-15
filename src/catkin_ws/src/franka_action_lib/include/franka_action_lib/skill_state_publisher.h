#ifndef FRANKA_ACTION_LIB_SKILL_STATE_PUBLISHER_H
#define FRANKA_ACTION_LIB_SKILL_STATE_PUBLISHER_H

#include <iostream>
#include <thread>
#include <array>
#include <chrono>
#include <vector>
#include <ros/ros.h>
#include <franka_action_lib/SkillState.h> // Note: "Action" is appended

#include "franka_action_lib/shared_memory_handler.h"

namespace franka_action_lib  
{ 
  class SkillStatePublisher
  {
    protected:

      ros::NodeHandle nh_;
      ros::Publisher skill_state_pub_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
      std::string topic_name_;

      double publish_frequency_;
      franka_action_lib::SharedMemoryHandler shared_memory_handler_;
      
      bool has_seen_one_skill_state_;
      franka_action_lib::SkillState last_skill_state_;

    public:

      SkillStatePublisher(std::string name);

      ~SkillStatePublisher(void){}

  };
}

#endif // FRANKA_ACTION_LIB_SKILL_STATE_PUBLISHER_H