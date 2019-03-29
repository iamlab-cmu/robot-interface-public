//
// Created by kevin on 3/26/19.
//

#include "iam_robolib/trajectory_generator/sine_joint_trajectory_generator.h"

#include <cassert>
#include <iostream>
#include <memory.h>

void SineJointTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int num_params = static_cast<int>(params_[1]);

  if(num_params == 8) {
    run_time_ = static_cast<double>(params_[2]);
    for (int i = 0; i < joint_goal_.size(); i++) {
      joint_goal_[i] = static_cast<double>(params_[i + 3]);
    }
  }
  else {
    std::cout << "Incorrect number of params given: " << num_params << std::endl;
  }
}

void SineJointTrajectoryGenerator::initialize_trajectory() {
  // assert(false);
}

void SineJointTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state) {
  TrajectoryGenerator::initialize_initial_states(robot_state);
  joint_desired_ = robot_state.q_d;
  joint_initial_ = robot_state.q_d;
}

void SineJointTrajectoryGenerator::get_next_step() {
  t_ = std::min(std::max(time_ / run_time_, 0.0), 1.0);
  sine_t_ = ((std::sin((t_ * M_PI) - (M_PI / 2)) + 1) / 2);

  for (int i = 0; i < joint_desired_.size(); i++) {
    joint_desired_[i] = joint_initial_[i] + (joint_goal_[i] - joint_initial_[i]) * sine_t_;
  }
}
  