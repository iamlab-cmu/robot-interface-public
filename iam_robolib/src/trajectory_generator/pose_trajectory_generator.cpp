#include "iam_robolib/trajectory_generator/pose_trajectory_generator.h"

#include <iostream>

void PoseTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int num_params = static_cast<int>(params_[1]);

  // Time + Full Cartesian Pose (std::array<double,16>) was given
  if(num_params == 17) {
    run_time_ = static_cast<double>(params_[2]);

    for(int i = 0; i < 16; i++) {
      goal_pose_[i] = static_cast<double>(params_[3+i]);
    }
    Eigen::Affine3d goal_transform(Eigen::Matrix4d::Map(goal_pose_.data()));
    goal_position_ = Eigen::Vector3d(goal_transform.translation());
    goal_orientation_ = Eigen::Quaterniond(goal_transform.linear());
  } 
  // Time + x,y,z + quaternion was given
  else if(num_params == 8) {
    run_time_ = static_cast<double>(params_[2]);

    goal_position_[0] = static_cast<double>(params_[3]);
    goal_position_[1] = static_cast<double>(params_[4]);
    goal_position_[2] = static_cast<double>(params_[5]);

    std::array<double,4> goal_quaternion{};
    for(int i = 0; i < 4; i++) {
      goal_quaternion[i] = static_cast<double>(params_[6+i]);
    }
    goal_orientation_ = Eigen::Quaterniond(goal_quaternion[0], goal_quaternion[1], 
                                           goal_quaternion[2], goal_quaternion[3]);
  } 
  // Time + x,y,z + axis angle was given
  else if(num_params == 7) {
    run_time_ = static_cast<double>(params_[2]);

    goal_position_[0] = static_cast<double>(params_[3]);
    goal_position_[1] = static_cast<double>(params_[4]);
    goal_position_[2] = static_cast<double>(params_[5]);

    Eigen::Vector3d goal_axis_angle;
    for(int i = 0; i < 3; i++) {
      goal_axis_angle[i] = static_cast<double>(params_[6+i]);
    }

    double angle = goal_axis_angle.norm();
    double sin_angle_divided_by_2 = std::sin(angle/2);
    double cos_angle_divided_by_2 = std::cos(angle/2);

    goal_orientation_ = Eigen::Quaterniond(goal_axis_angle[0] * sin_angle_divided_by_2,
                                           goal_axis_angle[1] * sin_angle_divided_by_2,
                                           goal_axis_angle[2] * sin_angle_divided_by_2,
                                           cos_angle_divided_by_2);
  }
  else {
    std::cout << "Invalid number of params provided: " << num_params << std::endl;
  }
}

void PoseTrajectoryGenerator::initialize_trajectory() {
  // pass
}

void PoseTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state) {
  initialize_initial_and_desired_poses(robot_state, SkillType::ImpedanceControlSkill);
}

void PoseTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state,
                                                    SkillType skill_type) {
  initialize_initial_and_desired_poses(robot_state, skill_type);
}

void PoseTrajectoryGenerator::initialize_initial_and_desired_poses(const franka::RobotState &robot_state,
                                                                   SkillType skill_type) {
  switch(skill_type) {
    case SkillType::ImpedanceControlSkill:
      // Use O_T_EE as the initial pose for Impedance Control for safety reasons
      initial_pose_ = robot_state.O_T_EE;
      desired_pose_ = robot_state.O_T_EE;
      break;
    case SkillType::CartesianPoseSkill:
      // Use O_T_EE_c as the initial pose for Cartesian Pose Control to 
      // avoid trajectory discontinuity errors
      initial_pose_ = robot_state.O_T_EE_c;
      desired_pose_ = robot_state.O_T_EE_c;
      break;
    default:
      // Default to using O_T_EE as the initial pose for safety reasons 
      initial_pose_ = robot_state.O_T_EE;
      desired_pose_ = robot_state.O_T_EE;
  }

  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_pose_.data()));
  initial_position_ = Eigen::Vector3d(initial_transform.translation());
  initial_orientation_ = Eigen::Quaterniond(initial_transform.linear());
  desired_position_ = Eigen::Vector3d(initial_transform.translation());
  desired_orientation_ = Eigen::Quaterniond(initial_transform.linear());
}

void PoseTrajectoryGenerator::calculate_desired_pose() {
    Eigen::Affine3d desired_pose_affine = Eigen::Affine3d::Identity();
    desired_pose_affine.translate(desired_position_);
    // Normalize desired orientation quaternion to avoid precision issues
    desired_orientation_.normalize();
    desired_pose_affine.rotate(desired_orientation_);
    Eigen::Matrix4d desired_pose_matrix = desired_pose_affine.matrix();

    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            desired_pose_[4*i+j] = desired_pose_matrix(j,i); // Column wise
        }
    }
}

const std::array<double, 16>& PoseTrajectoryGenerator::get_desired_pose() const {
  return desired_pose_;
}

const Eigen::Vector3d& PoseTrajectoryGenerator::get_desired_position() const {
  return desired_position_;
}

const Eigen::Quaterniond& PoseTrajectoryGenerator::get_desired_orientation() const {
  return desired_orientation_;
}

const Eigen::Vector3d& PoseTrajectoryGenerator::get_goal_position() const {
  return goal_position_;
}

const Eigen::Quaterniond& PoseTrajectoryGenerator::get_goal_orientation() const {
  return goal_orientation_;
}