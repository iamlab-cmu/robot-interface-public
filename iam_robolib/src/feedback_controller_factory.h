#pragma once

#include "definitions.h"

class feedback_controller;

class feedback_controller_factory {
 public:
  feedback_controller_factory() {};

  /**
   * Get feedback controller for skill.
   *
   * @param memory_region  Region of the memory where the parameters
   * will be stored.
   * @return FeedbackController instance for this skill
   */
  feedback_controller* getFeedbackControllerForSkill(SharedBuffer buffer);

};

