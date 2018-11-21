//
// Created by mohit on 11/21/18.
//

#pragma once

#include "trajectory_generator.h"

class CounterTrajectoryGenerator : TrajectoryGenerator {
 public:

  void parse_parameters() override;

  void initialiize_trajectory() override;

  void get_next_step() override;

 private:
  int start_ = 0;
  int end_ = 0;
  int delta_ = 0;
  int current_val_ = 0;
};

