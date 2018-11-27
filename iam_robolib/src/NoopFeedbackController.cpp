//
// Created by mohit on 11/25/18.
//

#include "NoopFeedbackController.h"

#include <iostream>

void NoopFeedbackController::parse_parameters() {
  delta_ = params_[2];
  std::cout << "delta parsed in fbc " << delta_ << std::endl;
}

void NoopFeedbackController::initialize_controller() {
  // pass
}

void NoopFeedbackController::get_next_step() {
  // pass
}
