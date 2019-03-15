#include "franka_action_lib/skill_state_publisher.h"

namespace franka_action_lib
{
  SkillStatePublisher::SkillStatePublisher(std::string name) :  nh_("~"),
                                                                topic_name_(name),
                                                                shared_memory_handler_()
  {
    nh_.param("publish_frequency", publish_frequency_, (double) 100.0);
    skill_state_pub_ = nh_.advertise<franka_action_lib::SkillState>(topic_name_, 100);

    ROS_INFO("Skill State Publisher Started");

    ros::Rate loop_rate(publish_frequency_);
    while (ros::ok())
    {
        franka_action_lib::SkillState skill_state_ = shared_memory_handler_.getSkillState();

        if (skill_state_.is_fresh) {
          skill_state_pub_.publish(skill_state_);

          has_seen_one_skill_state_ = true;
          last_skill_state_ = skill_state_;
          last_skill_state_.is_fresh = false;
        } else {
          if (has_seen_one_skill_state_) {
            skill_state_pub_.publish(last_skill_state_);
          }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
  }
}
