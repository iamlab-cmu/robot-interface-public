//
// Created by mohit on 11/20/18.
//

#include "skill_info_manager.h"

#include "BaseSkill.h"

#include <cassert>

SkillInfoManager::SkillInfoManager() {
}

BaseSkill *SkillInfoManager::get_current_skill() {
  if (skill_list_.size() == 0) {
    // returns NULL
    return 0;
  }
  return skill_list_.back();
}

bool SkillInfoManager::is_currently_executing_skill() {
  if (skill_list_.size() == 0){
    return false;
  }
  BaseSkill*skill = skill_list_.back();
  SkillStatus status = skill->get_current_skill_status();
  return (status == SkillStatus::TO_START or status == SkillStatus::RUNNING);
}

bool SkillInfoManager::is_waiting_for_new_skill() {
  if (skill_list_.size() == 0){
    return true;
  }
  return (*skill_list_.back()).get_current_skill_status() == SkillStatus::FINISHED;
}

void SkillInfoManager::add_skill(BaseSkill *skill) {
  assert(is_waiting_for_new_skill());
  skill_list_.push_back(skill);
}