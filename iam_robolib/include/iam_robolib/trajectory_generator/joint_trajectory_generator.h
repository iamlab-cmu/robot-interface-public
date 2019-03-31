#ifndef IAM_ROBOLIB_TRAJECTORY_GENERATOR_JOINT_TRAJECTORY_GENERATOR_H_
#define IAM_ROBOLIB_TRAJECTORY_GENERATOR_JOINT_TRAJECTORY_GENERATOR_H_

#include <array>

#include "iam_robolib/trajectory_generator/trajectory_generator.h"

class JointTrajectoryGenerator : public TrajectoryGenerator {
 public:
  using TrajectoryGenerator::TrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory() override;

  void initialize_trajectory(const franka::RobotState &robot_state) override;

  void initialize_trajectory(const franka::RobotState &robot_state,
                             SkillType skill_type) override;

  /**
   * Initialize initial and desired joints from robot state
   */
  void initialize_initial_and_desired_joints(const franka::RobotState &robot_state,
                                             SkillType skill_type);

  /**
   * Returns the desired joints
   */
  const std::array<double, 7>& get_desired_joints() const;

  /**
   * Returns the goal joints
   */
  const std::array<double, 7>& get_goal_joints() const;

 protected:
  std::array<double, 7> initial_joints_{};
  std::array<double, 7> desired_joints_{};
  std::array<double, 7> goal_joints_{};
};

#endif	// IAM_ROBOLIB_TRAJECTORY_GENERATOR_JOINT_TRAJECTORY_GENERATOR_H_