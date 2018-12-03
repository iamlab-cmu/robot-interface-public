#include "GripperOpenSkill.h"

#include <cassert>
#include <iostream>
#include <thread>

#include "GripperOpenTrajectoryGenerator.h"

void GripperOpenSkill::execute_skill() {
  assert(false);
}

void GripperOpenSkill::execute_skill_on_franka(franka::Robot *robot, franka::Gripper* gripper,
                                               ControlLoopData *control_loop_data) {
  // Check for the maximum grasping width.
  franka::GripperState gripper_state = gripper->readOnce();
  GripperOpenTrajectoryGenerator *gripper_traj_generator = static_cast<
      GripperOpenTrajectoryGenerator *>(traj_generator_);
  double open_width = gripper_traj_generator->getWidth();
  if (gripper_state.max_width < open_width) {
    std::cout << "Object is too large for the current fingers on the gripper." << std::endl;
    return_status_ = false;
    return;
  }

  double open_speed = gripper_traj_generator->getSpeed();
  if (gripper_traj_generator->isGraspSkill()) {
    return_status_ = gripper->grasp(open_width, open_speed, gripper_traj_generator->getForce());
  } else {
    return_status_ = gripper->move(open_width, open_speed);
  }

  // Block on this thread to allow gripper to execute skill.
  std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(
      gripper_traj_generator->getWaitTimeInMilliseconds()));
}

void GripperOpenSkill::execute_skill_on_franka_temp(franka::Robot *robot, franka::Gripper* gripper,
                                                    ControlLoopData *control_loop_data) {
  // Not implemented
  assert(false);
}

void GripperOpenSkill::execute_skill_on_franka_temp2(franka::Robot *robot, franka::Gripper* gripper,
                                                     ControlLoopData *control_loop_data) {
  // Not implemented
  assert(false);
}

bool GripperOpenSkill::should_terminate() {
  // Wait for some time before terminating this skill.
  return true;
}
