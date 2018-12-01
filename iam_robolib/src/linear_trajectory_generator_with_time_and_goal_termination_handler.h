#pragma once

#include <Eigen/Dense>
#include <TerminationHandler.h>

class LinearTrajectoryGeneratorWithTimeAndGoalTerminationHandler : public TerminationHandler {
 public:
  using TerminationHandler::TerminationHandler;

  /**
   * Parse parameters from memory.
   */
  void parse_parameters() override;

  /**
   * Initialize termination handler after parameter parsing.
   */
  void initialize_handler() override;

  /**
   * Should we terminate the current skill.
   */
  bool should_terminate(TrajectoryGenerator *traj_generator) override;

  /**
   * Should we terminate the current skill.
   */
  virtual bool should_terminate(franka::RobotState *robot_state, TrajectoryGenerator *traj_generator) override;

 private:
  int num_params_;
  double position_threshold_;
  double orientation_threshold_;
  Eigen::Vector3d position_thresholds_;
  Eigen::Vector3d orientation_thresholds_;
};
