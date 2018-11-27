//
// Created by mohit on 11/21/18.
//

#pragma once

#include "trajectory_generator.h"

class CounterTrajectoryGenerator : public TrajectoryGenerator {
 public:
  using TrajectoryGenerator::TrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory() override;

  void get_next_step() override;

  float current_point_[3] = {};
  int delta_ = 0;

 private:
  int start_ = 0;
  int end_ = 0;
  int current_val_ = 0;

  float start_point_[3] = {};
};

