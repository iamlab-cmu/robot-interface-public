//
// Created by mohit on 11/30/18.
//

#include "FinalJointTerminationHandler.h"

#include <iostream>

#include "LinearJointTrajectoryController.h"

void FinalJointTerminationHandler::parse_parameters() {
  // First parameter is reserved for the type

  int num_params = static_cast<int>(params_[1]);

  if(num_params != 7) {
    std::cout << "Incorrect number of params given: " << num_params << std::endl;
  }

  for (size_t i=0; i < joint_final_.size(); i++) {
    joint_final_[i] = static_cast<double>(params_[2 + i]);
  }
}

void FinalJointTerminationHandler::initialize_handler() {
  // pass
}

bool FinalJointTerminationHandler::should_terminate(TrajectoryGenerator *trajectory_generator) {
  LinearJointTrajectoryGenerator *linear_traj_generator =
      static_cast<LinearJointTrajectoryGenerator *>(trajectory_generator);
  for(size_t i = 0; i < joint_final_.size(); i++) {
    if(fabs(joint_final_[i] - linear_traj_generator->joint_desired_[i]) > 0.0001) {
      return false;
    }
  }
  return true;
}

bool FinalJointTerminationHandler::should_terminate(franka::RobotState *robot_state, TrajectoryGenerator *trajectory_generator) {
  LinearJointTrajectoryGenerator *linear_traj_generator =
      static_cast<LinearJointTrajectoryGenerator *>(trajectory_generator);
  for(size_t i = 0; i < joint_final_.size(); i++) {
    if(fabs(joint_final_[i] - linear_traj_generator->joint_desired_[i]) > 0.0001) {
      return false;
    }
  }
  return true;
}

