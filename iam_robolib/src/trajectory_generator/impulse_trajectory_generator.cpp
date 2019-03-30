//
// Created by jacky on 1/26/19.
//

#include "iam_robolib/trajectory_generator/impulse_trajectory_generator.h"

#include <cassert>
#include <iostream>
#include <memory.h>

void ImpulseTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int num_params = static_cast<int>(params_[1]);
  if (num_params == 10) {
    run_time_ = static_cast<double>(params_[2]);
    acc_time_ = static_cast<double>(params_[3]);
    max_translation_ = static_cast<double>(params_[4]);
    max_rotation_ = static_cast<double>(params_[5]);
    for (int i = 0; i < target_force_torque_.size(); i++) {
      target_force_torque_[i] = static_cast<double>(params_[i + 6]);
    }

  } else {
    std::cout << "Incorrect number of params given: " << num_params << std::endl;
  }
}

void ImpulseTrajectoryGenerator::initialize_trajectory() {
  // pass
}

void ImpulseTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state) {
  TrajectoryGenerator::initialize_initial_states(robot_state);
}

void ImpulseTrajectoryGenerator::get_next_step() {
  t_ = time_;

  double coef = 0;
  if (!should_deacc_) {
    if (t_ >= 0 && t_ < acc_time_) {
      coef = t_/acc_time_;
    } else if (t_ >= acc_time_ && t_ < run_time_ - acc_time_) {
      coef = 1.;
    } else if (t_ >= run_time_ - acc_time_ && t_ < run_time_) {
      coef = (run_time_ - t_)/acc_time_;
    } else {
      coef = 0.;
    }
  }

  for (int i = 0; i < target_force_torque_.size(); i++) {
    desired_force_torque_[i] = coef * target_force_torque_[i];
  }
}

void ImpulseTrajectoryGenerator::check_displacement_cap(const franka::RobotState& robot_state) {
  // check if max translation and rotation caps are reached
  if (!should_deacc_) {
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    
    if (max_translation_ > 0) {
      curr_position_ = transform.translation();
      if ((curr_position_ - initial_position_).norm() > max_translation_) {
        should_deacc_ = true;
      }
    }

    if (max_rotation_ > 0) {
      curr_orientation_ = transform.linear();
      if (curr_orientation_.coeffs().dot(initial_orientation_.coeffs()) < 0.0) {
        curr_orientation_.coeffs() << -curr_orientation_.coeffs();
      }
      Eigen::Quaterniond Q_delta(initial_orientation_ * curr_orientation_.inverse());
      Eigen::AngleAxisd A_delta(Q_delta);
      if (A_delta.angle() > max_rotation_) {
        should_deacc_ = true;
      }
    }
  }
}
  