//
// Created by mohit on 11/25/18.
//

#pragma once

#include <franka/robot.h>
#include "trajectory_generator.h"

class TerminationHandler {
 public:
  explicit TerminationHandler(float *p) : params_{p} {};

  /**
   * Parse parameters from memory.
   */
  virtual void parse_parameters() = 0;

  /**
   * Initialize termination handler after parameter parsing.
   */
  virtual void initialize_handler() = 0;

  /**
   * Should we terminate the current skill.
   */
  virtual bool should_terminate(TrajectoryGenerator *traj_generator) = 0;

  /**
   * Should we terminate the current skill.
   */
  virtual bool should_terminate(franka::RobotState *robot_state, TrajectoryGenerator *traj_generator) = 0;

 protected:
  float *params_=0;
};
