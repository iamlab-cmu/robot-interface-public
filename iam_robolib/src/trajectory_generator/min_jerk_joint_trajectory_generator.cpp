//
// Created by jacky on 2/11/19.
// From https://mika-s.github.io/python/control-theory/trajectory-generation/2017/12/06/trajectory-generation-with-a-minimum-jerk-trajectory.html
//

#include "iam_robolib/trajectory_generator/min_jerk_joint_trajectory_generator.h"

#include <cassert>
#include <iostream>
#include <memory.h>

void MinJerkJointTrajectoryGenerator::parse_parameters() {
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

void MinJerkJointTrajectoryGenerator::initialize_trajectory() {
  // assert(false);
}

void MinJerkJointTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state) {
  TrajectoryGenerator::initialize_initial_states(robot_state);
  joint_desired_ = robot_state.q_d;
  joint_initial_ = robot_state.q_d;
}

void MinJerkJointTrajectoryGenerator::get_next_step() {
  t_ = std::min(std::max(time_ / run_time_, 0.0), 1.0);
  
  for (int i = 0; i < joint_desired_.size(); i++) {
    joint_desired_[i] = joint_initial_[i] + (joint_goal_[i] - joint_initial_[i]) * (
            10 * std::pow(t_, 3) - 15 * std::pow(t_, 4) + 6 * std::pow(t_, 5)
        );
  }
}
  