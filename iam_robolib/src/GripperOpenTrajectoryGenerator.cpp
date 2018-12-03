//
// Created by iamlab on 12/1/18.
//

#include "GripperOpenTrajectoryGenerator.h"

#include <iostream>

void GripperOpenTrajectoryGenerator::parse_parameters() {
  int num_params = static_cast<int>(params_[1]);

  if(num_params != 3) {
    std::cout << "Incorrect number of params given: " << num_params << std::endl;
  }

  open_width_ = static_cast<double >(params_[2]);
  open_speed_ = static_cast<double >(params_[3]);
  wait_time_in_milliseconds_ = static_cast<double>(params_[4]);
}

void GripperOpenTrajectoryGenerator::initialize_trajectory() {
  // pass
}

void GripperOpenTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state) {
  // pass
}

void GripperOpenTrajectoryGenerator::get_next_step() {
  // pass
}

double GripperOpenTrajectoryGenerator::getOpenWidth() {
  return open_width_;
}

double GripperOpenTrajectoryGenerator::getOpenSpeed()  {
  return open_speed_;
}

double GripperOpenTrajectoryGenerator::getWaitTimeInMilliseconds() {
  return wait_time_in_milliseconds_;
}
