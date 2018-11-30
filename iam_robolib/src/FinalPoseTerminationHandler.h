#pragma once

#include <TerminationHandler.h>

#include <franka/robot.h>

class FinalPoseTerminationHandler : public TerminationHandler {
 public:
  // using TerminationHandler::TerminationHandler;

  FinalPoseTerminationHandler(float *p) :
      TerminationHandler(p),
      pose_final_(std::array<double, 16>()) {}

  /**
   * Parse parameters from memory.
   */
  void parse_parameters() override;

  /**
   * Initialize termination handler after parameter parsing.
   */
  void initialize_handler() override;

  /**
   * Should we terminate the current skill.
   */
  bool should_terminate(TrajectoryGenerator *traj_generator) override;

 private:
  franka::CartesianPose pose_final_{};
};
