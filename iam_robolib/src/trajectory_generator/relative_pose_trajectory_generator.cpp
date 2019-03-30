#include "iam_robolib/trajectory_generator/relative_pose_trajectory_generator.h"

#include <iostream>

void RelativePoseTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int num_params = static_cast<int>(params_[1]);

  // Time + Full Cartesian Pose (std::array<double,16>) was given
  if(num_params == 17) {
    run_time_ = static_cast<double>(params_[2]);

    for(int i = 0; i < 16; i++) {
      goal_pose_[i] = static_cast<double>(params_[3+i]);
    }
    Eigen::Affine3d goal_transform(Eigen::Matrix4d::Map(goal_pose_.data()));
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

void RelativePoseTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state) {
  initialize_initial_and_desired_poses(robot_state, SkillType::ImpedanceControlSkill);
  goal_position_ = initial_position_ + relative_position_;
  goal_orientation_ = initial_orientation_ * relative_orientation_;
}

void RelativePoseTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state,
                                                            SkillType skill_type) {
  initialize_initial_and_desired_poses(robot_state, skill_type);
  goal_position_ = initial_position_ + relative_position_;
  goal_orientation_ = initial_orientation_ * relative_orientation_;
}