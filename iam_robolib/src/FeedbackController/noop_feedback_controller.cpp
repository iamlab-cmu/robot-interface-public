//
// Created by mohit on 11/25/18.
//

#include "noop_feedback_controller.h"

#include <iostream>

void noop_feedback_controller::parse_parameters() {
  // pass
}

void noop_feedback_controller::initialize_controller() {
  // pass
}

void noop_feedback_controller::initialize_controller(franka::Model *model) {
  // pass
}

void noop_feedback_controller::get_next_step() {
  // pass
}

void noop_feedback_controller::get_next_step(const franka::RobotState &robot_state, TrajectoryGenerator *traj_generator) {
  // pass
}