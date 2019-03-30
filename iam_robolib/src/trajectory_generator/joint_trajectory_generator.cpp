#include "iam_robolib/trajectory_generator/joint_trajectory_generator.h"

#include <iostream>

void JointTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int num_params = static_cast<int>(params_[1]);

  if(num_params == 8) {
    run_time_ = static_cast<double>(params_[2]);
    for (size_t i = 0; i < goal_joints_.size(); i++) {
      goal_joints_[i] = static_cast<double>(params_[i + 3]);
    }
  }
  else {
    std::cout << "Incorrect number of params given: " << num_params << std::endl;
  }
}

void JointTrajectoryGenerator::initialize_trajectory() {
  // pass
}

void JointTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state) {
  initialize_initial_and_desired_joints(robot_state);
}

void JointTrajectoryGenerator::initialize_initial_and_desired_joints(const franka::RobotState &robot_state) {
  initial_joints_ = robot_state.q_d;
  desired_joints_ = robot_state.q_d;
}

const std::array<double, 7>& JointTrajectoryGenerator::get_desired_joints() const {
  return desired_joints_;
}

const std::array<double, 7>& JointTrajectoryGenerator::get_goal_joints() const {
  return goal_joints_;
}