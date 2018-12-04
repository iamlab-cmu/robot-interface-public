#pragma once

#include "trajectory_generator.h"

#include <array>
#include <cstring>
#include <iostream>
#include <franka/robot.h>
#include <Eigen/Dense>
#include <array>

class DMPTrajectoryGenerator : public TrajectoryGenerator {
 public:
  using TrajectoryGenerator::TrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory() override;

  void initialize_trajectory(const franka::RobotState &robot_state) override;

  void get_next_step() override;


 private:
  double alpha_=25.0;
  double beta_=25.0/4.0;
  double tau_=0.0;
  double x_=1.0;
  int num_basis_;
  int num_dims_=7;
  int num_sensor_values_=10;
  std::array<double, 10> basis_mean_{};
  std::array<double, 10> basis_std_{};
  std::array<std::array<std::array<double, 7>, 10>, 20> weights_{};
  std::array<double, 10> initial_sensor_values_{{1., 0., 0., 0., 1., 1., 0., 0., 1., 0.}};
  std::array<double, 7> y0_={};
  std::array<double, 7> y_={};
  std::array<double, 7> dy_={};
};
