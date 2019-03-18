#ifndef IAM_ROBOLIB_TRAJECTORY_GENERATOR_GRIPPER_TRAJECTORY_GENERATOR_H_
#define IAM_ROBOLIB_TRAJECTORY_GENERATOR_GRIPPER_TRAJECTORY_GENERATOR_H_

#include "iam_robolib/trajectory_generator/trajectory_generator.h"

/**
 * Used in Gripper skill. Specifies 3 parameters in the following order
 *
 *      1) gripper width
 *      2) gripper speed
 *      3) wait time for gripper to  in milliseconds.
 */
class GripperTrajectoryGenerator : public TrajectoryGenerator {
 public:
  using TrajectoryGenerator::TrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory() override;

  void initialize_trajectory(const franka::RobotState &robot_state) override;

  void get_next_step() override;

  /**
   * Get width to  the gripper to.
   * @return width speed.
   */
  double getWidth();

  /**
   * Get speed to  the gripper.
   * @return gripper speed.
   */
  double getSpeed();

  /**
   * Get Force to grasp an object.
   * @return
   */
  double getForce();

  /**
   * Check if the skill requires to grasp the object.
   * @return True if the skill requires to grasp the object, returns false if it does not.
   */
  bool isGraspSkill();

 private:
  double width_=1.0;
  double speed_=0.0;
  double force_=0.0;
  bool is_grasp_skill_{false};
};

#endif  // IAM_ROBOLIB_TRAJECTORY_GENERATOR_GRIPPER_TRAJECTORY_GENERATOR_H_