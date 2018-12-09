#pragma once

#include "BaseSkill.h"

class JointPoseWithTorqueControlSkill : public BaseSkill {
 public:
  JointPoseWithTorqueControlSkill(int skill_idx): BaseSkill(skill_idx) {};

  void execute_skill() override;

  void execute_skill_on_franka(franka::Robot* robot, franka::Gripper* gripper,
                               ControlLoopData *control_loop_data) override;

  void execute_continuous_skill_on_franka(franka::Robot* robot, franka::Gripper* gripper,
                                          ControlLoopData *control_loop_data);

 private:
  bool return_status_{false};
};

