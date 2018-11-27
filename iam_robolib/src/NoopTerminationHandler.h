//
// Created by mohit on 11/26/18.
//

#pragma once

#include "TerminationHandler.h"

class NoopTerminationHandler : public TerminationHandler {
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
  bool should_terminate() override;
};
