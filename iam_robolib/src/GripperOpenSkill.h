#pragma once

#include "skill_info.h"

class TrajectoryGenerator;
class FeedbackController;
class TerminationHandler;

class GripperOpenSkill : public SkillInfo {
 public:
  GripperOpenSkill(int skill_idx): SkillInfo(skill_idx) {};

  void execute_skill();

  void execute_skill_on_franka(franka::Robot* robot, franka::Gripper* gripper, ControlLoopData *control_loop_data);

  void execute_skill_on_franka_temp(franka::Robot* robot, franka::Gripper* gripper, ControlLoopData *control_loop_data);

  void execute_skill_on_franka_temp2(franka::Robot* robot, franka::Gripper* gripper, ControlLoopData *control_loop_data);

  bool should_terminate();

 private:
  bool return_status_{false};
};

