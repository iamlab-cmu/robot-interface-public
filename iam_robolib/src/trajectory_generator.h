//
// Created by mohit on 11/21/18.
//

#pragma once

#include <array>

class TrajectoryGenerator {
 public:
  explicit TrajectoryGenerator(std::array<float, 1024> p) : params_{p} {};

  /**
   * Parse parameters from memory.
   */
  virtual void parse_parameters() = 0;

  /**
   * Initialize trajectory generation after parameter parsing.
   */
  virtual void initialize_trajectory() = 0;

  /**
   *  Get next trajectory step.
   */
  virtual void get_next_step() = 0;

 protected:
  std::array<float, 1024> params_;

};

