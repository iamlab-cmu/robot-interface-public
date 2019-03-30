#ifndef IAM_ROBOLIB_TRAJECTORY_GENERATOR_POSE_TRAJECTORY_GENERATOR_H_
#define IAM_ROBOLIB_TRAJECTORY_GENERATOR_POSE_TRAJECTORY_GENERATOR_H_

#include <array>
#include <Eigen/Dense>
#include <iam_robolib_common/definitions.h>

#include "iam_robolib/trajectory_generator/trajectory_generator.h"

class PoseTrajectoryGenerator : public TrajectoryGenerator {
 public:
  using TrajectoryGenerator::TrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory() override;

  void initialize_trajectory(const franka::RobotState &robot_state) override;

  virtual void initialize_trajectory(const franka::RobotState &robot_state,
                                     SkillType skill_type);

  /**
   * Initialize initial and desired positions and orientations from robot state
   */
  void initialize_initial_and_desired_poses(const franka::RobotState &robot_state,
                                            SkillType skill_type);

  /**
   * Calculate desired pose using the desired position and orientation
   */
  void calculate_desired_pose();

  /**
   * Returns the desired pose
   */
  const std::array<double, 16>& get_desired_pose() const;

  /**
   * Returns the desired position
   */
  const Eigen::Vector3d& get_desired_position() const;

  /**
   * Returns the desired orientation
   */
  const Eigen::Quaterniond& get_desired_orientation() const;

  /**
   * Returns the goal position
   */
  const Eigen::Vector3d& get_goal_position() const;

  /**
   * Returns the goal orientation
   */
  const Eigen::Quaterniond& get_goal_orientation() const;

 protected:
  std::array<double, 16> initial_pose_{};
  std::array<double, 16> desired_pose_{};
  std::array<double, 16> goal_pose_{};

  Eigen::Vector3d initial_position_;
  Eigen::Quaterniond initial_orientation_;
  Eigen::Vector3d desired_position_;
  Eigen::Quaterniond desired_orientation_;
  Eigen::Vector3d goal_position_;
  Eigen::Quaterniond goal_orientation_;
};

#endif  // IAM_ROBOLIB_TRAJECTORY_GENERATOR_POSE_TRAJECTORY_GENERATOR_H_