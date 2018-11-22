//
// Created by mohit on 11/20/18.
//

#include "skill_info.h"

#include <cassert>

int SkillInfo::get_skill_id() {
    return skill_idx_;
}

void SkillInfo::set_skill_status(SkillStatus status) {
  // TODO(Mohit): Maybe add checks such that task status progresses
  // in one direction.
  skill_status_ = status;
}

void SkillInfo::start_skill(TrajectoryGenerator *traj_generator) {
  skill_status_ = SkillStatus::TO_START;
  traj_generator_ = traj_generator;
  traj_generator_->initialize_trajectory();
}

SkillStatus SkillInfo::get_current_skill_status() {
    return skill_status_;
}

void SkillInfo::execute_skill() {
  assert(traj_generator_ != 0);
  traj_generator_->get_next_step();
}

