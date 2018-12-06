#pragma once

#include "BaseSkill.h"

class JointPoseSkill : public BaseSkill {
 public:
  JointPoseSkill(int skill_idx): BaseSkill(skill_idx) {};

  void execute_skill();

  void execute_skill_on_franka(franka::Robot* robot, franka::Gripper* gripper, ControlLoopData *control_loop_data);

 private:
  bool return_status_{false};
};
