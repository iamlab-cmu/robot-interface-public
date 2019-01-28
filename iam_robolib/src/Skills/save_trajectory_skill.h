#pragma once

#include "base_skill.h"
#include <thread>

class SaveTrajectorySkill : public BaseSkill {
 public:
  SaveTrajectorySkill(int skill_idx, int meta_skill_idx, std::string description): BaseSkill(
      skill_idx, meta_skill_idx, description) {};

  void execute_skill() override;

  void execute_skill_on_franka(franka::Robot* robot, franka::Gripper* gripper,
                               RobotStateData *robot_state_data) override;

  bool should_terminate() override;

 private:
  bool return_status_{false};
  // Not using a mutex is ok, in the worst case we will just lose 1 or 2 data points. Hopefully, nothing bad.
  bool running_skill_{false};
  std::thread save_traj_thread_{};
};

