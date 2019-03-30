#include "iam_robolib/trajectory_generator/pose_trajectory_generator.h"

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
      break;
    case SkillType::CartesianPoseSkill:
      // Use O_T_EE_c as the initial pose for Cartesian Pose Control to 
      // avoid trajectory discontinuity errors
      initial_pose_ = robot_state.O_T_EE_c;
      break;
    default:
      // Default to using O_T_EE as the initial pose for safety reasons 
      initial_pose_ = robot_state.O_T_EE;
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