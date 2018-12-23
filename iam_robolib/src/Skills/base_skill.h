#pragma once

#include <franka/gripper.h>
#include <franka/robot.h>

enum class SkillStatus { TO_START, RUNNING, FINISHED };  // enum class

class control_loop_data;
class feedback_controller;
class termination_handler;
class TrajectoryGenerator;
namespace franka {
  class RobotState;
  class Duration;
}

class base_skill {
 public:
  base_skill(int skill_idx, int meta_skill_idx): skill_idx_(skill_idx),
                                                meta_skill_idx_(meta_skill_idx),
                                                skill_status_(SkillStatus::TO_START) {};

  /**
   * Get skill id.
   */
  int get_skill_id();

  /**
   * Get meta-skill id for this skill id.
   */
  int get_meta_skill_id();

  /**
   * Update skill status;
   */
  void set_skill_status(SkillStatus new_task_status);

  /**
   * Get current skill status.
   */
  SkillStatus get_current_skill_status();

  TrajectoryGenerator* get_trajectory_generator();
  feedback_controller* get_feedback_controller();
  termination_handler* get_termination_handler();

  /**
   * Start skill. Initiliazes and parses the parameters for different skill components.
   * @param robot
   * @param traj_generator
   * @param feedback_controller
   * @param termination_handler
   */
  void start_skill(franka::Robot* robot,
                   TrajectoryGenerator *traj_generator,
                   feedback_controller *feedback_controller,
                   termination_handler *termination_handler);

  virtual void execute_skill() = 0;

  /**
   * Execute skill on franka with the given robot and gripper configuration.
   * @param robot
   * @param gripper
   * @param control_loop_data
   */
  virtual void execute_skill_on_franka(franka::Robot *robot, franka::Gripper *gripper,
                                       control_loop_data *control_loop_data) = 0;

  virtual bool should_terminate();

  virtual void write_result_to_shared_memory(float *result_buffer);
  virtual void write_result_to_shared_memory(float *result_buffer, franka::Robot *robot);

  virtual void write_feedback_to_shared_memory(float *feedback_buffer);

 protected:
  int skill_idx_;
  int meta_skill_idx_;
  SkillStatus skill_status_;

  TrajectoryGenerator *traj_generator_= nullptr;
  feedback_controller *feedback_controller_= nullptr;
  termination_handler *termination_handler_= nullptr;
};
