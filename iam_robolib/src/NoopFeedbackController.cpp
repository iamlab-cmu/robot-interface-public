//
// Created by mohit on 11/25/18.
//

#include "NoopFeedbackController.h"

#include <iostream>

void NoopFeedbackController::parse_parameters() {
  delta_ = params_[2];
}

void NoopFeedbackController::initialize_controller() {
  // pass
}

void NoopFeedbackController::initialize_controller(franka::Model *model) {
  // pass
}

void NoopFeedbackController::get_next_step() {
  // pass
}

void NoopFeedbackController::get_next_step(const franka::RobotState &robot_state, TrajectoryGenerator *traj_generator) {
  // pass
}