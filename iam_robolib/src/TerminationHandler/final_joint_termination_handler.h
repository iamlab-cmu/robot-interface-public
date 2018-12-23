//
// Created by mohit on 11/30/18.
//

#pragma once

#include "termination_handler.h"

class final_joint_termination_handler :public termination_handler{
 public:
  using termination_handler::termination_handler;

  /**
   * Parse parameters from memory.
   */
  void parse_parameters() override;

  /**
   * Initialize termination handler after parameter parsing.
   */
  void initialize_handler() override;

  /**
   * Initialize termination handler after parameter parsing.
   */
  void initialize_handler(franka::Robot *robot) override;

  /**
   * Should we terminate the current skill.
   */
  bool should_terminate(TrajectoryGenerator *traj_generator) override;

  /**
   * Should we terminate the current skill.
   */
  bool should_terminate(const franka::RobotState &robot_state, TrajectoryGenerator *traj_generator) override;

 private:
  std::array<double, 16> joint_final_{};

};
