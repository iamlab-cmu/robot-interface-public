#ifndef IAM_ROBOLIB_TRAJECTORY_GENERATOR_RELATIVE_MIN_JERK_POSE_TRAJECTORY_GENERATOR_H_
#define IAM_ROBOLIB_TRAJECTORY_GENERATOR_RELATIVE_MIN_JERK_POSE_TRAJECTORY_GENERATOR_H_

#include <array>
#include <cstring>
#include <iostream>
#include <franka/robot.h>
#include <Eigen/Dense>

#include "iam_robolib/trajectory_generator/min_jerk_pose_trajectory_generator.h"

class RelativeMinJerkPoseTrajectoryGenerator : public MinJerkPoseTrajectoryGenerator {
 public:
  using MinJerkPoseTrajectoryGenerator::MinJerkPoseTrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory() override;

  void initialize_trajectory(const franka::RobotState &robot_state) override;

  void get_next_step() override;

 private:
  Eigen::Vector3d relative_position_;
  Eigen::Quaterniond relative_orientation_;
  
};

#endif  // IAM_ROBOLIB_TRAJECTORY_GENERATOR_RELATIVE_MIN_JERK_POSE_TRAJECTORY_GENERATOR_H_