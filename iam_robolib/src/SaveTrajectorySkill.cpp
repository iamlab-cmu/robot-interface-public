//
// Created by mohit on 12/6/18.
//

#include "SaveTrajectorySkill.h"

#include <cassert>
#include <franka/robot.h>
#include <iostream>

#include "ControlLoopData.h"
#include "TerminationHandler.h"

void SaveTrajectorySkill::execute_skill() {
  assert(false);
}

void SaveTrajectorySkill::execute_skill_on_franka(franka::Robot *robot, franka::Gripper* gripper,
                                               ControlLoopData *control_loop_data) {

  int print_rate = 30.0;
  double time;
  franka::Duration start_time;

  running_skill_ = true;
  save_traj_thread_ = std::thread([=, &print_rate, &time, &start_time]() {
    // Sleep to achieve the desired print rate.
    franka::RobotState robot_state = robot->readOnce();
    while (running_skill_) {
      if (time == 0.0) {
        start_time = robot_state.time;
      }

      control_loop_data->log_pose_desired(traj_generator_->pose_desired_);
      control_loop_data->log_robot_state(robot_state, time);

      time = (robot_state.time - start_time).toSec();

      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));
      std::cout << "will wait for " << static_cast<int>(1.0 / print_rate * 1000.0) << "\n";
    }
  });
}

bool SaveTrajectorySkill::should_terminate() {
  bool should_terminate = termination_handler_->should_terminate(traj_generator_);
  if (should_terminate) {
    running_skill_ = false;
    save_traj_thread_.join();
  }
  return should_terminate;
}
