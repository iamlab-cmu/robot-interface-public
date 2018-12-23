#pragma once

#include "skill_info.h"

class TrajectoryGenerator;
class feedback_controller;
class termination_handler;

class gripper_open_skill : public base_skill {
 public:
  gripper_open_skill(int skill_idx, int meta_skill_idx): base_skill(skill_idx, meta_skill_idx) {};

  void execute_skill() override;

  void execute_skill_on_franka(franka::Robot* robot, franka::Gripper* gripper,
                               control_loop_data *control_loop_data) override;

  bool should_terminate() override;

 private:
  bool return_status_{false};
};

