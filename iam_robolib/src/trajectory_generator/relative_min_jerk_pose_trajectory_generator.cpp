//
// Created by Kevin on 3/25/19.
// From https://mika-s.github.io/python/control-theory/trajectory-generation/2017/12/06/trajectory-generation-with-a-minimum-jerk-trajectory.html
//

#include "iam_robolib/trajectory_generator/relative_min_jerk_pose_trajectory_generator.h"

#include <cassert>
#include <iostream>
#include <memory.h>

void RelativeMinJerkPoseTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int num_params = static_cast<int>(params_[1]);

  // Time + Full Cartesian Pose (std::array<double,16>) was given
  if(num_params == 17) {
    run_time_ = static_cast<double>(params_[2]);

    std::array<double,16> cartesian_pose_goal{};
    for(int i = 0; i < 16; i++) {
      cartesian_pose_goal[i] = static_cast<double>(params_[3+i]);
    }
    Eigen::Affine3d goal_transform(Eigen::Matrix4d::Map(cartesian_pose_goal.data()));
    relative_position_ = Eigen::Vector3d(goal_transform.translation());
    relative_orientation_ = Eigen::Quaterniond(goal_transform.linear());
  } 
  // Time + x,y,z + quaternion was given
  else if(num_params == 8) {
    run_time_ = static_cast<double>(params_[2]);

    relative_position_[0] = static_cast<double>(params_[3]);
    relative_position_[1] = static_cast<double>(params_[4]);
    relative_position_[2] = static_cast<double>(params_[5]);

    std::array<double,4> goal_quaternion{};
    for(int i = 0; i < 4; i++) {
      goal_quaternion[i] = static_cast<double>(params_[6+i]);
    }
    relative_orientation_ = Eigen::Quaterniond(goal_quaternion[0], goal_quaternion[1], 
                                               goal_quaternion[2], goal_quaternion[3]);
  } 
  // Time + x,y,z + axis angle was given
  else if(num_params == 7) {
    run_time_ = static_cast<double>(params_[2]);

    relative_position_[0] = static_cast<double>(params_[3]);
    relative_position_[1] = static_cast<double>(params_[4]);
    relative_position_[2] = static_cast<double>(params_[5]);

    Eigen::Vector3d goal_axis_angle;
    for(int i = 0; i < 3; i++) {
      goal_axis_angle[i] = static_cast<double>(params_[6+i]);
    }

    double angle = goal_axis_angle.norm();
    double sin_angle_divided_by_2 = std::sin(angle/2);
    double cos_angle_divided_by_2 = std::cos(angle/2);

    relative_orientation_ = Eigen::Quaterniond(goal_axis_angle[0] * sin_angle_divided_by_2,
                                           goal_axis_angle[1] * sin_angle_divided_by_2,
                                           goal_axis_angle[2] * sin_angle_divided_by_2,
                                           cos_angle_divided_by_2);
  }
  else
  {
    std::cout << "Invalid number of params provided: " << num_params << std::endl;
  }
}


void RelativeMinJerkPoseTrajectoryGenerator::initialize_trajectory() {
  // assert(false);
}

void RelativeMinJerkPoseTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state) {
  TrajectoryGenerator::initialize_initial_states(robot_state);
  goal_position_ = initial_position_ + relative_position_;
  goal_orientation_ = initial_orientation_ * relative_orientation_;
}

void RelativeMinJerkPoseTrajectoryGenerator::get_next_step() {
  t_ = std::min(std::max(time_ / run_time_, 0.0), 1.0);
  
  for (int i = 0; i < desired_position_.size(); i++) {
    desired_position_[i] = initial_position_[i] + (goal_position_[i] - initial_position_[i]) * (
            10 * std::pow(t_, 3) - 15 * std::pow(t_, 4) + 6 * std::pow(t_, 5)
        );
  }

  slerp_t_ = (10 * std::pow(t_, 3) - 15 * std::pow(t_, 4) + 6 * std::pow(t_, 5));
  desired_orientation_ = initial_orientation_.slerp(slerp_t_, goal_orientation_);

  TrajectoryGenerator::calculate_desired_pose();
}
  