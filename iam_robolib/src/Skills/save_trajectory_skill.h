#pragma once

#include "base_skill.h"
#include <thread>

class save_trajectory_skill : public base_skill {
 public:
  save_trajectory_skill(int skill_idx, int meta_skill_idx): base_skill(skill_idx, meta_skill_idx) {};

  void execute_skill() override;

  void execute_skill_on_franka(franka::Robot* robot, franka::Gripper* gripper,
                               control_loop_data *control_loop_data) override;

  bool should_terminate() override;

 private:
  bool return_status_{false};
  // Not using a mutex is ok, in the worst case we will just lose 1 or 2 data points. Hopefully, nothing bad.
  bool running_skill_{false};
  std::thread save_traj_thread_{};
};

