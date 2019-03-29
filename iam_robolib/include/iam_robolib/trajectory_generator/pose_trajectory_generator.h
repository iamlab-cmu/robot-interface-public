#ifndef IAM_ROBOLIB_TRAJECTORY_GENERATOR_POSE_TRAJECTORY_GENERATOR_H_
#define IAM_ROBOLIB_TRAJECTORY_GENERATOR_POSE_TRAJECTORY_GENERATOR_H_

#include <array>
#include <cstring>
#include <iostream>
#include <franka/robot.h>
#include <Eigen/Dense>

#include "iam_robolib/trajectory_generator/trajectory_generator.h"

class PoseTrajectoryGenerator : public TrajectoryGenerator {
 public:
  using TrajectoryGenerator::TrajectoryGenerator;

  Eigen::Vector3d goal_position_;
  Eigen::Quaterniond goal_orientation_;
};

#endif  // IAM_ROBOLIB_TRAJECTORY_GENERATOR_POSE_TRAJECTORY_GENERATOR_H_