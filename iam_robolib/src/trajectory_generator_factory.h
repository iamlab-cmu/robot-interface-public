#pragma once

#include "run_loop_shared_memory_handler.h"
class TrajectoryGenerator;

typedef float* SharedBuffer;

class trajectory_generator_factory {
 public:

  trajectory_generator_factory() {};

  /**
   * Get trajectory generator for skill.
   *
   * @param memory_region  Region of the memory where the parameters
   * will be stored.
   */
  TrajectoryGenerator* getTrajectoryGeneratorForSkill(SharedBuffer buffer);

};

