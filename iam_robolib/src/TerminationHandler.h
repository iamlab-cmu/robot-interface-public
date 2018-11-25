//
// Created by mohit on 11/25/18.
//

#pragma once

#include "skill_info.h"

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
  virtual bool should_terminate(SkillInfo *skill_info) = 0;

 protected:
  float *params_=0;
};
