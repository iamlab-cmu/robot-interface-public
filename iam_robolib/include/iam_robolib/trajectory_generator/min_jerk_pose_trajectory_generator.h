#ifndef IAM_ROBOLIB_TRAJECTORY_GENERATOR_MIN_JERK_POSE_TRAJECTORY_GENERATOR_H_
#define IAM_ROBOLIB_TRAJECTORY_GENERATOR_MIN_JERK_POSE_TRAJECTORY_GENERATOR_H_

#include "iam_robolib/trajectory_generator/pose_trajectory_generator.h"

class MinJerkPoseTrajectoryGenerator : public PoseTrajectoryGenerator {
 public:
  using PoseTrajectoryGenerator::PoseTrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory() override;

  void initialize_trajectory(const franka::RobotState &robot_state) override;

  void get_next_step() override;

  double slerp_t_ = 0.0;
};

#endif	// IAM_ROBOLIB_TRAJECTORY_GENERATOR_MIN_JERK_POSE_TRAJECTORY_GENERATOR_H_