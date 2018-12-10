#pragma once

#include "BaseMetaSkill.h"

class RunLoop;

class JointPoseContinuousSkill : public BaseMetaSkill {
 public:
  JointPoseContinuousSkill(int skill_idx): BaseMetaSkill(skill_idx) {
    is_composable_ = true;
  };

  void execute_skill_on_franka(RunLoop *run_loop, franka::Robot* robot, franka::Gripper* gripper,
                               ControlLoopData *control_loop_data) override;

 private:
  bool return_status_{false};
};
