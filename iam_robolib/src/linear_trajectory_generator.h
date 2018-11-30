//
// Created by Kevin on 11/29/18.
//

#pragma once

#include <cstring>
#include <iostream>
#include <franka/robot.h>

#include "trajectory_generator.h"

class LinearTrajectoryGenerator : public TrajectoryGenerator {
 public:
  using TrajectoryGenerator::TrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory() override;

  void initialize_trajectory(franka::RobotState robot_state) override;

  void get_next_step() override;

 private:
  const float vel_max_ = 0.25;
  float time_ = 0.0;
  float deltas_[16]={};
  float jerk_ = 0.0;
  float acceleration_ = 0.0;
  float velocity_ = 0.0;
};

