#include <ros/ros.h>
#include "franka_action_lib/skill_state_publisher.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "skill_state_publisher_node", ros::init_options::AnonymousName);

  franka_action_lib::SkillStatePublisher skill_state_publisher("skill_state");

  return 0;
}