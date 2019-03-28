#ifndef IAM_ROBOLIB_TRAJECTORY_GENERATOR_LINEAR_POSE_TRAJECTORY_GENERATOR_H_
#define IAM_ROBOLIB_TRAJECTORY_GENERATOR_LINEAR_POSE_TRAJECTORY_GENERATOR_H_

#include <array>
#include <cstring>
#include <iostream>
#include <franka/robot.h>
#include <Eigen/Dense>

#include "iam_robolib/trajectory_generator/pose_trajectory_generator.h"

class LinearPoseTrajectoryGenerator : public PoseTrajectoryGenerator {
 public:
  using PoseTrajectoryGenerator::PoseTrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory() override;

  void initialize_trajectory(const franka::RobotState &robot_state) override;

  void get_next_step() override;

  bool saved_full_trajectory_ = true;
  
};

#endif  // IAM_ROBOLIB_TRAJECTORY_GENERATOR_LINEAR_POSE_TRAJECTORY_GENERATOR_H_