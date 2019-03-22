//
// Created by mohit on 12/18/18.
//

#include "iam_robolib/trajectory_generator_factory.h"

#include <iostream>
#include <iam_robolib_common/definitions.h>

#include "iam_robolib/skills/base_meta_skill.h"
#include "iam_robolib/skills/base_skill.h"
#include "iam_robolib/trajectory_generator/dmp_trajectory_generator.h"
#include "iam_robolib/trajectory_generator/gripper_trajectory_generator.h"
#include "iam_robolib/trajectory_generator/counter_trajectory_generator.h"
#include "iam_robolib/trajectory_generator/linear_trajectory_generator.h"
#include "iam_robolib/trajectory_generator/linear_trajectory_generator_with_time_and_goal.h"
#include "iam_robolib/trajectory_generator/linear_joint_trajectory_generator.h"
#include "iam_robolib/trajectory_generator/min_jerk_joint_trajectory_generator.h"
#include "iam_robolib/trajectory_generator/relative_linear_trajectory_generator.h"
#include "iam_robolib/trajectory_generator/stay_in_initial_position_trajectory_generator.h"
#include "iam_robolib/trajectory_generator/impulse_trajectory_generator.h"

TrajectoryGenerator* TrajectoryGeneratorFactory::getTrajectoryGeneratorForSkill(
    SharedBufferTypePtr buffer) {
  TrajectoryGeneratorType trajectory_generator_type = static_cast<TrajectoryGeneratorType>(buffer[0]);
  TrajectoryGenerator *trajectory_generator = nullptr;

  std::cout << "Trajectory Generator Type: " << 
  static_cast<std::underlying_type<TrajectoryGeneratorType>::type>(trajectory_generator_type) << 
  "\n";

  switch (trajectory_generator_type) {
    case TrajectoryGeneratorType::CounterTrajectoryGenerator:
      trajectory_generator = new CounterTrajectoryGenerator(buffer);
      break;
    case TrajectoryGeneratorType::LinearTrajectoryGenerator:
      trajectory_generator = new LinearTrajectoryGenerator(buffer);
      break;
    case TrajectoryGeneratorType::LinearJointTrajectoryGenerator:
      trajectory_generator = new LinearJointTrajectoryGenerator(buffer);
      break;
    case TrajectoryGeneratorType::LinearTrajectoryGeneratorWithTimeAndGoal:
      trajectory_generator = new LinearTrajectoryGeneratorWithTimeAndGoal(buffer);
      break;
    case TrajectoryGeneratorType::GripperTrajectoryGenerator:
      trajectory_generator = new GripperTrajectoryGenerator(buffer);
      break;
    case TrajectoryGeneratorType::StayInInitialPositionTrajectoryGenerator:
      trajectory_generator = new StayInInitialPositionTrajectoryGenerator(buffer);
      break;
    case TrajectoryGeneratorType::DmpTrajectoryGenerator:
      trajectory_generator = new DmpTrajectoryGenerator(buffer);
      break;
    case TrajectoryGeneratorType::RelativeLinearTrajectoryGenerator:
      trajectory_generator = new RelativeLinearTrajectoryGenerator(buffer);
      break;
    case TrajectoryGeneratorType::ImpulseTrajectoryGenerator:
      trajectory_generator = new ImpulseTrajectoryGenerator(buffer);
      break;
    case TrajectoryGeneratorType::MinJerkJointTrajectoryGenerator:
      trajectory_generator = new MinJerkJointTrajectoryGenerator(buffer);
      break;
    default:
      std::cout << "Cannot create Trajectory Generator with type:" << 
      static_cast<std::underlying_type<TrajectoryGeneratorType>::type>(trajectory_generator_type) << 
      "\n";
      return nullptr;
  }

  trajectory_generator->parse_parameters();
  return trajectory_generator;
}

