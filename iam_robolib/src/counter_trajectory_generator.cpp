//
// Created by mohit on 11/21/18.
//

#include "counter_trajectory_generator.h"

void CounterTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type
  start_ = int(params_[1]);
  end_ = int(params_[2]);
  delta_ = int(params_[3]);
}

void CounterTrajectoryGenerator::initialiize_trajectory() {
  current_val_ = start_;
}

void CounterTrajectoryGenerator::get_next_step() {
  current_val_ = current_val_ + delta_;
}

