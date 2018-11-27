//
// Created by mohit on 11/25/18.
//

#pragma once

#include "FeedbackController.h"

class NoopFeedbackController : public FeedbackController {
 public:
  using FeedbackController::FeedbackController;

  void parse_parameters() override;

  void initialize_controller() override;

  void get_next_step() override;

  float delta_=0.0;
};
