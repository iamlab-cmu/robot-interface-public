#ifndef IAM_ROBOLIB_TRAJECTORY_GENERATOR_RELATIVE_LINEAR_POSE_TRAJECTORY_GENERATOR_H_
#define IAM_ROBOLIB_TRAJECTORY_GENERATOR_RELATIVE_LINEAR_POSE_TRAJECTORY_GENERATOR_H_

#include <array>
#include <cstring>
#include <iostream>
#include <franka/robot.h>
#include <Eigen/Dense>

#include "iam_robolib/trajectory_generator/relative_pose_trajectory_generator.h"

class RelativeLinearPoseTrajectoryGenerator : public RelativePoseTrajectoryGenerator {
 public:
  using RelativePoseTrajectoryGenerator::RelativePoseTrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory() override;

  void initialize_trajectory(const franka::RobotState &robot_state) override;

  void get_next_step() override;

};

#endif  // IAM_ROBOLIB_TRAJECTORY_GENERATOR_RELATIVE_LINEAR_POSE_TRAJECTORY_GENERATOR_H_