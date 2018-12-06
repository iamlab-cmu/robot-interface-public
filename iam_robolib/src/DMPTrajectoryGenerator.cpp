//
// Created by mohit on 12/3/18.
//

#include "DMPTrajectoryGenerator.h"

#include <cstdlib>
#include <cmath>
#include <iostream>

void DMPTrajectoryGenerator::parse_parameters() {
  int num_params = static_cast<int>(params_[1]);

  // Time + Full Cartesian Pose (std::array<double,16>) was given
  if(num_params == 1411) {
    tau_ = params_[2];
    num_basis_ = static_cast<int>(params_[3]);
    num_sensor_values_ = static_cast<int>(params_[4]);
    memcpy(&y0_, &params_[4], 7 * sizeof(float));

    memcpy(&weights_, &params_[4+7], 7 * 20 * 10 * sizeof(float));

    // Set the mean and std for basis function.
    for (int i = 0; i < num_basis_; i++)  {
      basis_mean_[i] = exp(-(i)*0.5/(num_basis_-1));
    }
    for (int i = 0; i < num_basis_ - 1; i++) {
      basis_std_[i] = 0.5 / (0.65 * pow(basis_mean_[i+1] - basis_mean_[i], 2));
    }
    basis_std_[num_basis_ - 1] = basis_std_[num_basis_ - 2];

    std::array<double, 7> y_={};
    std::array<double, 7> dy_={};
  } else {
    std::cout << "DMPTrajectoryGenerator Invalid number of parameters: " << num_params <<
      std::endl;
  }
}

void DMPTrajectoryGenerator::initialize_trajectory() {
  // assert(false);
}

void DMPTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state) {
  // TODO: Should we use desired joint values here?
  y_ = robot_state.q;
  dy_ = robot_state.dq;
  x_ = 1.0;
}


void DMPTrajectoryGenerator::get_next_step() {
  static int i, j, k;
  static double ddy, t;

  static std::array<double, 20> factor{};
  static double den = 0.;
  static double net_sensor_force;
  static double sensor_feature;

  // Calculate feature values.
  den = 0.;
  for (k = 0; k < num_basis_ - 1; k++) {
    factor[k] = exp(-basis_std_[k] * pow((x_ -basis_mean_[k]), 2));
    den += factor[k];
  }

  for (k=0; k < num_basis_ - 1; k++) {
    factor[k] = (factor[k] * x_) / (den * basis_mean_[k]);
  }
  t = fmin(-log(x_)/tau_, 1);
  factor[num_basis_ - 1] = pow(t, 3) * (6*pow(t, 2) - 15 * t + 10);

  for (i=0; i < num_dims_; i++) {
    ddy = (alpha_ * (beta_ * (y0_[i] - y_[i]) - tau_ * dy_[i]));
    net_sensor_force = 0;
    for (j = 0; j < num_sensor_values_; j++) {
      sensor_feature = 0;
      for (k=0; k < num_basis_; k++) {
        sensor_feature += (factor[k] * weights_[i][j][k]);
      }
      net_sensor_force += (initial_sensor_values_[j] * sensor_feature);
    }
    ddy += (alpha_ * beta_ * net_sensor_force);
    dy_[i] += (ddy / (tau_*tau_)) * dt_;
    y_[i] += (dy_[i] * dt_);
  }

  // Update canonical system.
  x_ -= (x_ * tau_) * dt_;

  // Finally set the joints we want.
  joint_desired_ = dy_;
}

