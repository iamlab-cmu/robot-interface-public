//
// Created by mohit on 11/30/18.
//

#include "LinearJointTrajectoryController.h"

#include <cassert>
#include <iostream>
#include <memory.h>

void LinearJointTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int num_params = static_cast<int>(params_[1]);

  if(num_params != 7) {
    std::cout << "Incorrect number of params given: " << num_params << std::endl;
  }

  memcpy(deltas_, &params_[2], 7 * sizeof(float));
}

void LinearJointTrajectoryGenerator::initialize_trajectory() {
  // assert(false);
}

void LinearJointTrajectoryGenerator::initialize_trajectory(franka::RobotState robot_state) {
  joint_desired_ = robot_state.q;
}

void LinearJointTrajectoryGenerator::get_next_step() {
  double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * time_));
  joint_desired_[3] += delta_angle;
  joint_desired_[4] += delta_angle;
  joint_desired_[6] += delta_angle;
}

