#pragma once

#include "trajectory_generator.h"

/**
 * Used in GripperOpen skill. Specifies 3 parameters in the following order
 *
 *      1) gripper open width
 *      2) gripper open speed
 *      3) wait time for gripper to open in milliseconds.
 */
class GripperOpenTrajectoryGenerator : public TrajectoryGenerator {
 public:
  using TrajectoryGenerator::TrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory() override;

  void initialize_trajectory(const franka::RobotState &robot_state) override;

  void get_next_step() override;

  double getOpenWidth();
  double getOpenSpeed();
  double getWaitTimeInMilliseconds();

 private:
  double open_width_=1.0;
  double open_speed_=0.0;
  double wait_time_in_milliseconds_=3000.0;

};

