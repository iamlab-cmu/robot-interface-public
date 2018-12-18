#pragma once

#include "SharedMemoryHandler.h"
class TrajectoryGenerator;

typedef float* SharedBuffer;

class TrajectoryGeneratorFactory {
 public:

  TrajectoryGeneratorFactory() {};

  /**
   * Get trajectory generator for skill.
   *
   * @param memory_region  Region of the memory where the parameters
   * will be stored.
   */
  TrajectoryGenerator* getTrajectoryGeneratorForSkill(SharedBuffer buffer);

};

