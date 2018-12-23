//
// Created by mohit on 12/9/18.
//

#include "base_meta_skill.h"

#include <iam_robolib/run_loop.h>

int base_meta_skill::getMetaSkillId() {
  return skill_idx_;
}

bool base_meta_skill::isComposableSkill() {
  return is_composable_;
}

void base_meta_skill::setMetaSkillStatus(SkillStatus new_status) {
  skill_status_ = new_status;
}

SkillStatus base_meta_skill::getCurrentMetaSkillStatus() {
  return skill_status_;
}

void base_meta_skill::execute_skill_on_franka(run_loop* run_loop, franka::Robot *robot,
    franka::Gripper *gripper, control_loop_data *control_loop_data) {
  base_skill* skill = run_loop->getSkillInfoManager()->get_current_skill();
  if (skill != nullptr) {
    skill->execute_skill_on_franka(robot, gripper, control_loop_data);
    run_loop->finish_current_skill(skill);
  }
}
