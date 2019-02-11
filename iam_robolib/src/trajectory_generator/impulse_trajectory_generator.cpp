//
// Created by jacky on 1/26/19.
//

#include "iam_robolib/trajectory_generator/impulse_trajectory_generator.h"

#include <cassert>
#include <iostream>
#include <memory.h>

void ImpulseTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int num_params = static_cast<int>(params_[1]);
  if (num_params == 8) {
    run_time_ = static_cast<double>(params_[2]);
    acc_time_ = static_cast<double>(params_[3]);
    for (int i = 0; i < force_torque_target_.size(); i++) {
      force_torque_target_[i] = static_cast<double>(params_[i + 4]);
    }
  } else {
    std::cout << "Incorrect number of params given: " << num_params << std::endl;
  }
}

void ImpulseTrajectoryGenerator::initialize_trajectory() {
  // pass
}

void ImpulseTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state) {
  // pass
}

void ImpulseTrajectoryGenerator::get_next_step() {
  t_ = time_;

  double coef;
  if (t_ >= 0 && t_ < acc_time_) {
    coef = t_/acc_time_;
  } else if (t_ >= acc_time_ && t_ < run_time_ - acc_time_) {
    coef = 1.;
  } else if (t_ >= run_time_ - acc_time_ && t_ < run_time_) {
    coef = (run_time_ - t_)/acc_time_;
  } else {
    coef = 0.;
  }

  for (int i = 0; i < force_torque_target_.size(); i++) {
      force_torque_desired_[i] = coef * force_torque_target_[i];
  }
}
  