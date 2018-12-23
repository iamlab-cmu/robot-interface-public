#pragma once

#include "base_meta_skill.h"

class run_loop;

class joint_pose_continuous_skill : public base_meta_skill {
 public:
  joint_pose_continuous_skill(int skill_idx): base_meta_skill(skill_idx) {
    is_composable_ = true;
  };

  bool isComposableSkill() override;
  
  void execute_skill_on_franka(run_loop *run_loop, franka::Robot* robot, franka::Gripper* gripper,
                               control_loop_data *control_loop_data) override;

 private:
  bool return_status_{false};
};
