#pragma once

#include "base_skill.h"

class JointPoseWithTorqueControlSkill : public BaseSkill {
 public:
  JointPoseWithTorqueControlSkill(int skill_idx, int meta_skill_idx): BaseSkill(skill_idx, meta_skill_idx) {};

  void execute_skill() override;

  void execute_skill_on_franka(franka::Robot* robot, franka::Gripper* gripper,
                               RobotStateData *robot_state_data) override;

  void execute_meta_skill_on_franka(franka::Robot *robot, franka::Gripper *gripper,
                                    RobotStateData *robot_state_data);

 private:
  bool return_status_{false};
};

