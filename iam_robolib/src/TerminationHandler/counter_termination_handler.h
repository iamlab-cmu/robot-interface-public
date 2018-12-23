//
// Created by mohit on 11/25/18.
//

#pragma once

#include "termination_handler.h"

class counter_termination_handler : public termination_handler {
 public:
  using termination_handler::termination_handler;

  void parse_parameters() override;

  void initialize_handler() override;

  void initialize_handler(franka::Robot *robot) override;

  bool should_terminate(TrajectoryGenerator *traj_generator) override;

  bool should_terminate(const franka::RobotState &robot_state, TrajectoryGenerator *traj_generator) override;

 private:
  int start_ = 0;
  int end_ = 0;
  int delta_ = 0;
  int current_val_ = 0;
};
