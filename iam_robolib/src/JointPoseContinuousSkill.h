#pragma once

#include "BaseSkill.h"

class JointPoseContinuousSkill : public BaseSkill {
 public:
  JointPoseContinuousSkill(int skill_idx): BaseSkill(skill_idx) {};

  void execute_skill() override;

  void execute_skill_on_franka(franka::Robot* robot, franka::Gripper* gripper,
                               ControlLoopData *control_loop_data) override;

 private:
  bool return_status_{false};

};
