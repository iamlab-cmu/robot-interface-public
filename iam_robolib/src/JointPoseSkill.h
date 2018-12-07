#pragma once

#include "BaseSkill.h"

class JointPoseSkill : public BaseSkill {
 public:
  JointPoseSkill(int skill_idx): BaseSkill(skill_idx) {};

  void execute_skill() override;

  void execute_skill_on_franka(franka::Robot* robot, franka::Gripper* gripper, ControlLoopData *control_loop_data) override;

 private:
  bool return_status_{false};
};
