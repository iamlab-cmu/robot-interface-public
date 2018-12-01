//
// Created by mohit on 11/26/18.
//

#include "NoopTerminationHandler.h"

#include <cmath>
#include <iostream>

#include "trajectory_generator.h"
#include "counter_trajectory_generator.h"

void NoopTerminationHandler::parse_parameters() {
  end_point_[0] = params_[2];
  end_point_[1] = params_[3];
  end_point_[2] = params_[4];
  std::cout << "Traj end point: " << end_point_[0] << ", " << end_point_[1] << ", "
            << end_point_[2] << std::endl;
}

void NoopTerminationHandler::initialize_handler() {
  // pass
}

bool NoopTerminationHandler::should_terminate(TrajectoryGenerator *trajectory_generator) {
  CounterTrajectoryGenerator *counter_traj_generator = static_cast<CounterTrajectoryGenerator
      *>(trajectory_generator);
  if (fabs(end_point_[0] - counter_traj_generator->current_point_[0]) < 0.0001 &&
      fabs(end_point_[1] - counter_traj_generator->current_point_[1]) < 0.0001 &&
      fabs(end_point_[2] - counter_traj_generator->current_point_[2]) < 0.0001) {
    return true;
  }
  return false;
}

bool NoopTerminationHandler::should_terminate(franka::RobotState *robot_state, TrajectoryGenerator *trajectory_generator) {
  CounterTrajectoryGenerator *counter_traj_generator = static_cast<CounterTrajectoryGenerator
      *>(trajectory_generator);
  if (fabs(end_point_[0] - counter_traj_generator->current_point_[0]) < 0.0001 &&
      fabs(end_point_[1] - counter_traj_generator->current_point_[1]) < 0.0001 &&
      fabs(end_point_[2] - counter_traj_generator->current_point_[2]) < 0.0001) {
    return true;
  }
  return false;
}
