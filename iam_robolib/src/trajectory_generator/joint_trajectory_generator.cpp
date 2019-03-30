#include "iam_robolib/trajectory_generator/joint_trajectory_generator.h"

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