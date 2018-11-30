//
// Created by Kevin on 11/29/18.
//

#include "linear_trajectory_generator.h"

#include <cassert>

void LinearTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int num_params = static_cast<int>(params_[1]);

  if(num_params != 16) {
    std::cout << "Incorrect number of params given: " << num_params << std::endl;
  }

  memcpy(deltas_, &params_[2], 16 * sizeof(float));
  assert(deltas_[13] < 5.0 && deltas_[14] < 5.0);
}

void LinearTrajectoryGenerator::initialize_trajectory() {
  assert(false);
}

void LinearTrajectoryGenerator::initialize_trajectory(franka::RobotState robot_state) {
  pose_desired_ = robot_state.O_T_EE_c;
}

void LinearTrajectoryGenerator::get_next_step() {
  for(int i = 13; i < 15; i++) {
    pose_desired_[i] += deltas_[i];
  }
}

