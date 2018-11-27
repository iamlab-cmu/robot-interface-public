//
// Created by mohit on 11/20/18.
//

#include "skill_info.h"

#include <cassert>
#include <iostream>

int SkillInfo::get_skill_id() {
    return skill_idx_;
}

void SkillInfo::set_skill_status(SkillStatus status) {
  // TODO(Mohit): Maybe add checks such that task status progresses
  // in one direction.
  skill_status_ = status;
}

void SkillInfo::start_skill(TrajectoryGenerator *traj_generator,
                            FeedbackController *feedback_controller,
                            TerminationHandler *termination_handler) {
  skill_status_ = SkillStatus::TO_START;
  traj_generator_ = traj_generator;
  traj_generator_->initialize_trajectory();
  feedback_controller_ = feedback_controller;
  feedback_controller_->initialize_controller();
  termination_handler_ = termination_handler;
  termination_handler_->initialize_handler();
}

SkillStatus SkillInfo::get_current_skill_status() {
    return skill_status_;
}

void SkillInfo::execute_skill() {
  assert(traj_generator_ != 0);
  // HACK
  std::string skill_status_string = "Running";
  if (skill_status_ == SkillStatus::FINISHED) {
    skill_status_string = "Finished";
  }
  std::cout << "Will execute skill with status: " << skill_status_string << "\n";
  traj_generator_->get_next_step();
}

bool SkillInfo::should_terminate() {
  return termination_handler_->should_terminate(traj_generator_);
}

