#ifndef IAM_ROBOLIB_TRAJECTORY_GENERATOR_MIN_JERK_JOINT_TRAJECTORY_GENERATOR_H_
#define IAM_ROBOLIB_TRAJECTORY_GENERATOR_MIN_JERK_JOINT_TRAJECTORY_GENERATOR_H_

#include "iam_robolib/trajectory_generator/trajectory_generator.h"

class MinJerkJointTrajectoryGenerator : public TrajectoryGenerator {
 public:
  using TrajectoryGenerator::TrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory() override;

  void initialize_trajectory(const franka::RobotState &robot_state) override;

  void get_next_step() override;

  std::array<double, 7> joint_goal_={};
 private:
  std::array<double, 7> joint_initial_={};
  double t_ = 0.0;
};

#endif	// IAM_ROBOLIB_TRAJECTORY_GENERATOR_MIN_JERK_JOINT_TRAJECTORY_GENERATOR_H_