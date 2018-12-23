//
// Created by Kevin on 11/29/18.
//

#pragma once

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include "TrajectoryGenerator/trajectory_generator.h"


class LinearTrajectoryGenerator : public TrajectoryGenerator {
 public:
  using TrajectoryGenerator::TrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory() override;

  void get_next_step() override;

  std::function<Torques(const RobotState&, franka::Duration)> impedance_control_callback_;
  std::function<CartesianPose(const RobotState&, franka::Duration)> cartesian_pose_callback_;

  // Set gains for the joint impedance control.
  // Stiffness
  float k_gains_[7];
  // Damping
  float d_gains_[7];

  franka::Model model_;

  double time_ = 0.0; 
  bool running_;
  std::array<double, 16> initial_pose_;

  const int num_deltas_ = 16;
  const int num_gains_ = 7;

  float deltas_[16];

 private:

  franka::CartesianPose pose_desired_;
};

