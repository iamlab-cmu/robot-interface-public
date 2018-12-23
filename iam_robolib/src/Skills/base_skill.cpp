//
// Created by iamlab on 12/2/18.
//

#include "base_skill.h"

#include <franka/robot.h>
#include <iostream>

#include "FeedbackController/feedback_controller.h"
#include "TerminationHandler/termination_handler.h"
#include "TrajectoryGenerator/trajectory_generator.h"

int base_skill::get_skill_id() {
  return skill_idx_;
}

int base_skill::get_meta_skill_id() {
  return meta_skill_idx_;
}

void base_skill::set_skill_status(SkillStatus status) {
  // TODO(Mohit): Maybe add checks such that task status progresses
  // in one direction.
  skill_status_ = status;
}

SkillStatus base_skill::get_current_skill_status() {
  return skill_status_;
}

TrajectoryGenerator* base_skill::get_trajectory_generator() {
  return traj_generator_;
}

feedback_controller* base_skill::get_feedback_controller() {
  return feedback_controller_;
}

termination_handler* base_skill::get_termination_handler() {
  return termination_handler_;
}

void base_skill::start_skill(franka::Robot* robot,
                            TrajectoryGenerator *traj_generator,
                            feedback_controller *feedback_controller,
                            termination_handler *termination_handler) {
  skill_status_ = SkillStatus::TO_START;
  traj_generator_ = traj_generator;
  traj_generator_->initialize_trajectory();
  feedback_controller_ = feedback_controller;
  feedback_controller_->initialize_controller();
  termination_handler_ = termination_handler;
  termination_handler_->initialize_handler();
  termination_handler_->initialize_handler(robot);
}

bool base_skill::should_terminate() {
  return termination_handler_->should_terminate(traj_generator_);
}

void base_skill::write_result_to_shared_memory(float *result_buffer) {
  std::cout << "Should write result to shared memory\n";
}

void base_skill::write_result_to_shared_memory(float *result_buffer, franka::Robot* robot) {
  std::cout << "Should write result to shared memory\n";
}

void base_skill::write_feedback_to_shared_memory(float *feedback_buffer) {
  std::cout << "Should write feedback to shared memory\n";
}

