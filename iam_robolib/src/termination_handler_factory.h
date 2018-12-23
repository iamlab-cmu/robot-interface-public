#pragma once

class termination_handler;
typedef float * SharedBuffer;

class termination_handler_factory {
 public:
  termination_handler_factory() {};

  /**
   * Get termination handler for skill.
   *
   * @param memory_region  Region of the memory where the parameters
   * will be stored.
   * @return TermatinationHanndler instance for this skill
   */
  termination_handler* getTerminationHandlerForSkill(SharedBuffer buffer);

};
