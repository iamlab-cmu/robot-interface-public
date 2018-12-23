//
// Created by mohit on 12/18/18.
//

#include "feedback_controller_factory.h"

#include <iostream>

#include "FeedbackController/custom_gain_torque_controller.h"
#include "FeedbackController/noop_feedback_controller.h"
#include "FeedbackController/torque_feedback_controller.h"

feedback_controller* feedback_controller_factory::getFeedbackControllerForSkill(SharedBuffer buffer){
  int feedback_controller_id = static_cast<int>(buffer[0]);

  std::cout << "Feedback Controller id: " << feedback_controller_id << "\n";

  feedback_controller* feedback_controller = nullptr;
  if (feedback_controller_id == 1) {
    // Create Counter based trajectory.
    feedback_controller = new noop_feedback_controller(buffer);
  } else if (feedback_controller_id == 2) {
    // Create Counter based trajectory.
    feedback_controller = new TorqueFeedbackController(buffer);
  } else if (feedback_controller_id == 3) {
    feedback_controller = new custom_gain_torque_controller(buffer);
  } else {
    std::cout << "Cannot create feedback_controller with class_id: " << feedback_controller_id
      << "\n";
    return nullptr;
  }
  feedback_controller->parse_parameters();
  return feedback_controller;
}
