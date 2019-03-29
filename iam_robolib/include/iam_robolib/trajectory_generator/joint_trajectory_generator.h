#ifndef IAM_ROBOLIB_TRAJECTORY_GENERATOR_JOINT_TRAJECTORY_GENERATOR_H_
#define IAM_ROBOLIB_TRAJECTORY_GENERATOR_JOINT_TRAJECTORY_GENERATOR_H_

#include "iam_robolib/trajectory_generator/trajectory_generator.h"

class JointTrajectoryGenerator : public TrajectoryGenerator {
 public:
  using TrajectoryGenerator::TrajectoryGenerator;

  std::array<double, 7> joint_goal_={};
  std::array<double, 7> joint_initial_={};
};

#endif	// IAM_ROBOLIB_TRAJECTORY_GENERATOR_JOINT_TRAJECTORY_GENERATOR_H_