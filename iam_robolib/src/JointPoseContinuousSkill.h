#pragma once

#include "BaseMetaSkill.h"

class JointPoseContinuousSkill : public BaseMetaSkill {
 public:
  JointPoseContinuousSkill(int skill_idx): BaseMetaSkill(skill_idx) {};

  void execute_skill_on_franka(BaseSkill *skill, franka::Robot* robot, franka::Gripper* gripper,
                               ControlLoopData *control_loop_data) override;

 private:
  bool return_status_{false};

};
