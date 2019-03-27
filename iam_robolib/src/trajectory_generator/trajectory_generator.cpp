//
// Created by mohit on 11/21/18.
//

#include "iam_robolib/trajectory_generator/trajectory_generator.h"

#include <iostream>

void TrajectoryGenerator::initialize_initial_states(const franka::RobotState &robot_state) {
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE_c.data()));
  initial_position_ = Eigen::Vector3d(initial_transform.translation());
  initial_orientation_ = Eigen::Quaterniond(initial_transform.linear());
}

void TrajectoryGenerator::calculate_desired_pose() {
    Eigen::Affine3d desired_affine_pose = Eigen::Affine3d::Identity();
    desired_affine_pose.translate(desired_position_);
    desired_orientation_.normalize();
    desired_affine_pose.rotate(desired_orientation_);
    Eigen::Matrix4d desired_pose = desired_affine_pose.matrix();

    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            pose_desired_[4*i+j] = desired_pose(j,i); // Column wise
        }
    }
}
