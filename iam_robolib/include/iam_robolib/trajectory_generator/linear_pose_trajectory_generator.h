#ifndef IAM_ROBOLIB_TRAJECTORY_GENERATOR_LINEAR_POSE_TRAJECTORY_GENERATOR_H_
#define IAM_ROBOLIB_TRAJECTORY_GENERATOR_LINEAR_POSE_TRAJECTORY_GENERATOR_H_

#include <array>
#include <cstring>
#include <iostream>
#include <franka/robot.h>
#include <Eigen/Dense>

#include "iam_robolib/trajectory_generator/trajectory_generator.h"

class LinearPoseTrajectoryGenerator : public TrajectoryGenerator {
 public:
  using TrajectoryGenerator::TrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory() override;

  void initialize_trajectory(const franka::RobotState &robot_state) override;

  void get_next_step() override;

  Eigen::Vector3d goal_position_;
  Eigen::Quaterniond goal_orientation_;

 private:
  double t_ = 0.0;
};

#endif  // IAM_ROBOLIB_TRAJECTORY_GENERATOR_LINEAR_POSE_TRAJECTORY_GENERATOR_H_