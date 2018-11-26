//
// Created by mohit on 11/26/18.
//

#include "NoopTerminationHandler.h"

void NoopTerminationHandler::parse_parameters() {
  // pass
}

void NoopTerminationHandler::initialize_handler() {
  // pass
}

bool NoopTerminationHandler::should_terminate(SkillInfo *skill_info) {
  // Never Terminate (or some canonical condition).
  return false;
}
