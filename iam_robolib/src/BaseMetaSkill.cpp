//
// Created by mohit on 12/9/18.
//

#include "BaseMetaSkill.h"

int BaseMetaSkill::getMetaSkillId() {
  return skill_idx_;
}

void BaseMetaSkill::isComposableSkill() {
  return is_composable_;
}

void BaseMetaSkill::setMetaSkillStatus(SkillStatus new_status) {
  skill_status_ = new_status;
}

SkillStatus BaseMetaSkill::getCurrentMetaSkillStatus() {
  return skill_status_;
}

void BaseMetaSkill::execute_skill_on_franka(BaseSkill* skill, franka::Robot *robot,
    franka::Gripper *gripper, ControlLoopData *control_loop_data) {
  skill->execute_skill_on_franka(robot, gripper, control_loop_data);
}
