#ifndef IAM_ROBOLIB_TRAJECTORY_GENERATOR_GOAL_POSE_DMP_TRAJECTORY_GENERATOR_H_
#define IAM_ROBOLIB_TRAJECTORY_GENERATOR_GOAL_POSE_DMP_TRAJECTORY_GENERATOR_H_

#include "iam_robolib/trajectory_generator/pose_trajectory_generator.h"

class GoalPoseDmpTrajectoryGenerator : public PoseTrajectoryGenerator {
 public:
  using PoseTrajectoryGenerator::PoseTrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory(const franka::RobotState &robot_state) override;

  void initialize_trajectory(const franka::RobotState &robot_state,
                             SkillType skill_type) override;

  void get_next_step() override;

  std::array<double, 3> y_={};
  std::array<double, 3> dy_={};

  double min_z = 0.0308;
  double eps = -0.01;

 private:
  // Variables initialized from shared memory should be doubles.
  double alpha_=5.0;
  double beta_=5.0/4.0;
  double tau_=0.0;
  double x_=1.0;
  int num_basis_=6;
  int num_dims_=3;
  int num_sensor_values_=2;
  std::array<double, 6> basis_mean_{};
  std::array<double, 6> basis_std_{};
  std::array<std::array<std::array<double, 6>, 2>, 3> weights_{};
  std::array<std::array<double, 2>, 3> initial_sensor_values_{};
  std::array<double, 3> y0_={};

  void getInitialMeanAndStd();
};

#endif  // IAM_ROBOLIB_TRAJECTORY_GENERATOR_GOAL_POSE_DMP_TRAJECTORY_GENERATOR_H_