//
// Created by iamlab on 12/2/18.
//

#include "BaseSkill.h"

#include <franka/robot.h>
#include <iostream>

#include "FeedbackController.h"
#include "TerminationHandler.h"
#include "trajectory_generator.h"

int BaseSkill::get_skill_id() {
  return skill_idx_;
}

void BaseSkill::set_skill_status(SkillStatus status) {
  // TODO(Mohit): Maybe add checks such that task status progresses
  // in one direction.
  skill_status_ = status;
}

SkillStatus BaseSkill::get_current_skill_status() {
  return skill_status_;
}

void BaseSkill::start_skill(franka::Robot* robot,
                            TrajectoryGenerator *traj_generator,
                            FeedbackController *feedback_controller,
                            TerminationHandler *termination_handler) {
  skill_status_ = SkillStatus::TO_START;
  traj_generator_ = traj_generator;
  traj_generator_->initialize_trajectory();
  feedback_controller_ = feedback_controller;
  feedback_controller_->initialize_controller();
  termination_handler_ = termination_handler;
  termination_handler_->initialize_handler();
  termination_handler_->initialize_handler(robot);
}

bool BaseSkill::should_terminate() {
  return termination_handler_->should_terminate(traj_generator_);
}

void BaseSkill::write_result_to_shared_memory(float *result_buffer) {
  std::cout << "Should write result to shared memory\n";
}

void BaseSkill::write_result_to_shared_memory(float *result_buffer, franka::Robot* robot) {
  std::cout << "Should write result to shared memory\n";
}

void BaseSkill::write_feedback_to_shared_memory(float *feedback_buffer) {
  std::cout << "Should write feedback to shared memory\n";
}

