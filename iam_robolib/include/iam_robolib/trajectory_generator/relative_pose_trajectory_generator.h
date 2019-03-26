#ifndef IAM_ROBOLIB_TRAJECTORY_GENERATOR_RELATIVE_POSE_TRAJECTORY_GENERATOR_H_
#define IAM_ROBOLIB_TRAJECTORY_GENERATOR_RELATIVE_POSE_TRAJECTORY_GENERATOR_H_

#include <array>
#include <cstring>
#include <iostream>
#include <franka/robot.h>
#include <Eigen/Dense>

#include "iam_robolib/trajectory_generator/pose_trajectory_generator.h"

class RelativePoseTrajectoryGenerator : public PoseTrajectoryGenerator {
 public:
  using PoseTrajectoryGenerator::PoseTrajectoryGenerator;

  Eigen::Vector3d relative_position_;
  Eigen::Quaterniond relative_orientation_;
  
};

#endif  // IAM_ROBOLIB_TRAJECTORY_GENERATOR_RELATIVE_POSE_TRAJECTORY_GENERATOR_H_