//
// Created by mohit on 11/25/18.
//

#pragma once

#include "TerminationHandler.h"

class CounterTerminationHandler : public TerminationHandler {
 public:
  using TerminationHandler::TerminationHandler;

  void parse_parameters() override;

  void initialize_handler() override;

  bool should_terminate(TrajectoryGenerator *traj_generator) override;

  bool should_terminate(const franka::RobotState &robot_state, TrajectoryGenerator *traj_generator) override;

 private:
  int start_ = 0;
  int end_ = 0;
  int delta_ = 0;
  int current_val_ = 0;
};
