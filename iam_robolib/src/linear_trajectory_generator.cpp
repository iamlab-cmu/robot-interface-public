//
// Created by Kevin on 11/29/18.
//

#include "linear_trajectory_generator.h"

void LinearTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int num_params = static_cast<int>(params_[1]);

  if(num_params != 32)
  {
    std::cout << "Incorrect number of params given: " << num_params << std::endl;
  }

  memcpy(deltas_, &params_[2], 16 * sizeof(float));
}

void LinearTrajectoryGenerator::initialize_trajectory(franka::RobotState robot_state) {
  pose_desired_ = robot_state.O_T_EE_c;
}

void LinearTrajectoryGenerator::get_next_step() {
  for(int i = 0; i < 16; i++)
  {
    pose_desired_.O_T_EE[i] += deltas_[i];
  }
}

