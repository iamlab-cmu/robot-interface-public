//
// Created by mohit on 11/26/18.
//

#include "noop_termination_handler.h"

void noop_termination_handler::parse_parameters() {
  // pass
}

void noop_termination_handler::initialize_handler() {
  // pass
}

void noop_termination_handler::initialize_handler(franka::Robot *robot) {
  // pass
}

bool noop_termination_handler::should_terminate(TrajectoryGenerator *trajectory_generator) {
  // pass
}

bool noop_termination_handler::should_terminate(const franka::RobotState &robot_state, TrajectoryGenerator *trajectory_generator) {
  // pass
}
