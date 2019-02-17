//
// Created by jacky on 2/13/19.
//

#include "iam_robolib/feedback_controller/pass_through_feedback_controller.h"

#include <iostream>

void PassThroughFeedbackController::parse_parameters() {
  // pass
}

void PassThroughFeedbackController::initialize_controller() {
  // pass
}

void PassThroughFeedbackController::initialize_controller(franka::Model *model) {
  model_ = model;
}

void PassThroughFeedbackController::get_next_step() {
  // pass
}

void PassThroughFeedbackController::get_next_step(const franka::RobotState &robot_state,
                                                TrajectoryGenerator *traj_generator) {
    
    double* force_torque_desired_ptr = &(traj_generator->force_torque_desired_[0]);
    Eigen::Map<Eigen::VectorXd> desired_force_torque(force_torque_desired_ptr, 6);

    // get jacobian
    std::array<double, 42> jacobian_array = model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

    std::array<double, 7> coriolis_array = model_->coriolis(robot_state);
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

    // compute control torques
    Eigen::VectorXd tau_d(7);
    tau_d << jacobian.transpose() * desired_force_torque + coriolis;

    Eigen::VectorXd::Map(&tau_d_array_[0], 7) = tau_d;
}