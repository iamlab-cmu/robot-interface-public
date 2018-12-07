#pragma once

#include "BaseSkill.h"
#include <thread>

class SaveTrajectorySkill : public BaseSkill {
 public:
  SaveTrajectorySkill(int skill_idx): BaseSkill(skill_idx) {};

  void execute_skill();

  void execute_skill_on_franka(franka::Robot* robot, franka::Gripper* gripper, ControlLoopData *control_loop_data);

  bool should_terminate();

 private:
  bool return_status_{false};
  // Not using a mutex is ok, in the worst case we will just lose 1 or 2 data points. Hopefully, nothing bad.
  bool running_skill_{false};
  std::thread save_traj_thread_{};
};

