//
// Created by mohit on 11/25/18.
//

#pragma once

#include "feedback_controller.h"

class noop_feedback_controller : public feedback_controller {
 public:
  using feedback_controller::feedback_controller;

  void parse_parameters() override;

  void initialize_controller() override;

  void initialize_controller(franka::Model *model) override;

  void get_next_step() override;

  void get_next_step(const franka::RobotState &robot_state, TrajectoryGenerator *traj_generator) override;

  float delta_=0.0;
};
