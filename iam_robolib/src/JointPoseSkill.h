#pragma once

#include "BaseSkill.h"

class JointPoseSkill : public BaseSkill {
 public:
  JointPoseSkill(int skill_idx, int meta_skill_idx): BaseSkill(skill_idx, meta_skill_idx) {};

  void execute_skill() override;

  void execute_skill_on_franka(franka::Robot* robot, franka::Gripper* gripper,
                               ControlLoopData *control_loop_data) override;

  void execute_meta_skill_on_franka(franka::Robot *robot, franka::Gripper *gripper,
                                    ControlLoopData *control_loop_data) override;

  bool next_step_on_franka(const franka::RobotState& robot_state, franka::Duration period, double& time) override;

 private:
  bool return_status_{false};
};
