//
// Created by mohit on 11/21/18.
//

#pragma once

#include <array>
#include <franka/robot.h>

class TrajectoryGenerator {
 public:
  explicit TrajectoryGenerator(float *p) : params_{p} {};

  /**
   * Parse parameters from memory.
   */
  virtual void parse_parameters() = 0;

  /**
   * Initialize trajectory generation after parameter parsing.
   */
  virtual void initialize_trajectory() = 0;

  /**
   * Initialize trajectory generation after parameter parsing.
   */
  virtual void initialize_trajectory(franka::RobotState robot_state) = 0;

  /**
   *  Get next trajectory step.
   */
  virtual void get_next_step() = 0;


  franka::CartesianPose pose_desired_ = franka::CartesianPose(cartesian_pose);

 protected:
  float *params_=0;
  std::array<double, 16> cartesian_pose;

};

