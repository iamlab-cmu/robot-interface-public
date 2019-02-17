//
// Created by mohit on 11/21/18.
//

#include "iam_robolib/trajectory_generator/trajectory_generator.h"


void TrajectoryGenerator::initialize_initial_states(const franka::RobotState &robot_state) {
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  initial_position_ = Eigen::Vector3d(initial_transform.translation());
  initial_orientation_ = Eigen::Quaterniond(initial_transform.linear());
}
