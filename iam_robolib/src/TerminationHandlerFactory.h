#pragma once

class TerminationHandler;
typedef float * SharedBuffer;

class TerminationHandlerFactory {
 public:
  TerminationHandlerFactory() {};

  /**
   * Get termination handler for skill.
   *
   * @param memory_region  Region of the memory where the parameters
   * will be stored.
   * @return TermatinationHanndler instance for this skill
   */
  TerminationHandler* getTerminationHandlerForSkill(SharedBuffer buffer);

};
