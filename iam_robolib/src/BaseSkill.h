#pragma once

#include <franka/gripper.h>
#include <franka/robot.h>

enum class SkillStatus { TO_START, RUNNING, FINISHED };  // enum class

class ControlLoopData;
class FeedbackController;
class TerminationHandler;
class TrajectoryGenerator;

class BaseSkill {
 public:
  BaseSkill(int skill_idx): skill_idx_(skill_idx),
                            skill_status_(SkillStatus::TO_START) {};

  int get_skill_id();

  void set_skill_status(SkillStatus new_task_status);

  SkillStatus get_current_skill_status();

  void start_skill(franka::Robot* robot,
                   TrajectoryGenerator *traj_generator,
                   FeedbackController *feedback_controller,
                   TerminationHandler *termination_handler);

  virtual void execute_skill() = 0;

  virtual void execute_skill_on_franka(franka::Robot *robot, franka::Gripper *gripper,
                                       ControlLoopData *control_loop_data) = 0;

  virtual bool should_terminate();

  virtual void write_result_to_shared_memory(float *result_buffer);

  virtual void write_feedback_to_shared_memory(float *feedback_buffer);

 protected:
  int skill_idx_;
  SkillStatus skill_status_;

  TrajectoryGenerator *traj_generator_= nullptr;
  FeedbackController *feedback_controller_= nullptr;
  TerminationHandler *termination_handler_= nullptr;
};
