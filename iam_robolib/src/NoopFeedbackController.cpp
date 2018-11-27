//
// Created by mohit on 11/25/18.
//

#include "NoopFeedbackController.h"

void NoopFeedbackController::parse_parameters() {
  delta_ = params_[2];
}

void NoopFeedbackController::initialize_controller() {
  // pass
}

void NoopFeedbackController::get_next_step() {
  // pass
}
