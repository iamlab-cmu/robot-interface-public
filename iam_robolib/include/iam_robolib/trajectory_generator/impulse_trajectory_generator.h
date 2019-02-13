#ifndef IAM_ROBOLIB_TRAJECTORY_GENERATOR_IMPULSE_TRAJECTORY_GENERATOR_H_
#define IAM_ROBOLIB_TRAJECTORY_GENERATOR_IMPULSE_TRAJECTORY_GENERATOR_H_

#include "iam_robolib/trajectory_generator/trajectory_generator.h"

class ImpulseTrajectoryGenerator : public TrajectoryGenerator {
 public:
  using TrajectoryGenerator::TrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory() override;

  void initialize_trajectory(const franka::RobotState &robot_state) override;

  void get_next_step() override;

  void check_displacement_cap(const franka::RobotState &robot_state);

 private:
  double t_ = 0.;
  double acc_time_ = 0.;
  std::array<double, 6> force_torque_target_{};
  bool should_deacc_ = false;

  double max_translation_{0.};
  double max_rotation_{0.}; 
  Eigen::Vector3d Tr_init;
  Eigen::Quaterniond Q_init;
  Eigen::Vector3d Tr_curr;
  Eigen::Quaterniond Q_curr;
};

#endif	// IAM_ROBOLIB_TRAJECTORY_GENERATOR_IMPULSE_TRAJECTORY_GENERATOR_H_