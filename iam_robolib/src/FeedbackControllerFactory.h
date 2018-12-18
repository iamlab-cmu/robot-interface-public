#pragma once

class FeedbackController;
typedef float* SharedBuffer;

class FeedbackControllerFactory {
 public:
  FeedbackControllerFactory() {};

  /**
   * Get feedback controller for skill.
   *
   * @param memory_region  Region of the memory where the parameters
   * will be stored.
   * @return FeedbackController instance for this skill
   */
  FeedbackController* getFeedbackControllerForSkill(SharedBuffer buffer);

};

