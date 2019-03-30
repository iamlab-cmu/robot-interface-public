//
// Created by Kevin on 11/29/18.
//

#include "iam_robolib/trajectory_generator/stay_in_initial_pose_trajectory_generator.h"

#include <iostream>

void StayInInitialPoseTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int num_params = static_cast<int>(params_[1]);

  if(num_params == 0) {
    std::cout << "StayInInitialPoseTrajectoryGenerator: No parameters provided. Using default run_time. " << std::endl;
  } 
  // Time
  else if(num_params == 1) {
    run_time_ = static_cast<double>(params_[2]);
  } 
  else {
    std::cout << "StayInInitialPoseTrajectoryGenerator: Invalid number of params provided: " << num_params << std::endl;
  }
}

void StayInInitialPoseTrajectoryGenerator::get_next_step() {
  desired_position_ = initial_position_;
  desired_orientation_ = initial_orientation_;
}