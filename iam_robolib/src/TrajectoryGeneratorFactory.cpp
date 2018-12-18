//
// Created by mohit on 12/18/18.
//

#include "TrajectoryGeneratorFactory.h"

#include "BaseMetaSkill.h"
#include "BaseSkill.h"
#include "DMPTrajectoryGenerator.h"
#include "GripperOpenTrajectoryGenerator.h"
#include "LinearJointTrajectoryController.h"
#include "counter_trajectory_generator.h"
#include "goal_linear_trajectory_generator.h"
#include "linear_trajectory_generator.h"
#include "linear_trajectory_generator_with_time_and_goal.h"
#include "linear_trajectory_generator_with_time_and_goal_termination_handler.h"
#include "relative_linear_trajectory_generator.h"
#include "stay_in_initial_position_trajectory_generator.h"

TrajectoryGenerator* TrajectoryGeneratorFactory::getTrajectoryGeneratorForSkill(
    SharedBuffer buffer) {
  int traj_gen_id = static_cast<int>(buffer[0]);
  TrajectoryGenerator *traj_generator = nullptr;

  std::cout << "Trajectory generator id: " << traj_gen_id << "\n";

  if (traj_gen_id == 1) {
    // Create Counter based trajectory.
    traj_generator = new CounterTrajectoryGenerator(buffer);
  } else if (traj_gen_id == 2) {
    traj_generator = new LinearTrajectoryGenerator(buffer);
  } else if (traj_gen_id == 3) {
    traj_generator = new LinearJointTrajectoryGenerator(buffer);
  } else if (traj_gen_id == 4) {
    traj_generator = new LinearTrajectoryGeneratorWithTimeAndGoal(buffer);
  } else if (traj_gen_id == 5){
    traj_generator = new GripperOpenTrajectoryGenerator(buffer);
  } else if (traj_gen_id == 6) {
    traj_generator = new StayInInitialPositionTrajectoryGenerator(buffer);
  } else if (traj_gen_id == 7) {
    traj_generator = new DMPTrajectoryGenerator(buffer);
  } else if (traj_gen_id == 8) {
    traj_generator = new RelativeLinearTrajectoryGenerator(buffer);
  } else {
    // Cannot create Trajectory generator for this skill. Throw error
    std::cout << "Cannot create TrajectoryGenerator with class_id:" << traj_gen_id << "\n";
    return nullptr;
  }
  traj_generator->parse_parameters();
  return traj_generator;
}

