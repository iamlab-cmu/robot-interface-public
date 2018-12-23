#pragma once

#include "base_skill.h"

class joint_pose_with_torque_control_skill : public base_skill {
 public:
  joint_pose_with_torque_control_skill(int skill_idx, int meta_skill_idx): base_skill(skill_idx, meta_skill_idx) {};

  void execute_skill() override;

  void execute_skill_on_franka(franka::Robot* robot, franka::Gripper* gripper,
                               control_loop_data *control_loop_data) override;

  void execute_meta_skill_on_franka(franka::Robot *robot, franka::Gripper *gripper,
                                    control_loop_data *control_loop_data);

 private:
  bool return_status_{false};
};

