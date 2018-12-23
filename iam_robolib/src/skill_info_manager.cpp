//
// Created by mohit on 11/20/18.
//

#include "skill_info_manager.h"

#include "Skills/base_skill.h"
#include "Skills/base_meta_skill.h"

#include <cassert>

SkillInfoManager::SkillInfoManager() {
}

base_skill* SkillInfoManager::get_current_skill() {
  if (skill_list_.size() == 0) {
    // returns NULL
    return 0;
  }
  return skill_list_.back();
}

base_meta_skill* SkillInfoManager::get_current_meta_skill() {
  if (meta_skill_list_.size() == 0) {
    // returns NULL
    return 0;
  }
  return meta_skill_list_.back();
}

bool SkillInfoManager::is_currently_executing_skill() {
  if (skill_list_.size() == 0){
    return false;
  }
  base_skill*skill = skill_list_.back();
  SkillStatus status = skill->get_current_skill_status();
  return (status == SkillStatus::TO_START or status == SkillStatus::RUNNING);
}

bool SkillInfoManager::is_waiting_for_new_skill() {
  if (skill_list_.size() == 0){
    return true;
  }
  return (*skill_list_.back()).get_current_skill_status() == SkillStatus::FINISHED;
}

void SkillInfoManager::add_skill(base_skill *skill) {
  assert(is_waiting_for_new_skill());
  skill_list_.push_back(skill);
}

void SkillInfoManager::add_meta_skill(base_meta_skill *skill) {
  meta_skill_list_.push_back(skill);
}

base_meta_skill* SkillInfoManager::get_meta_skill_with_id(int meta_skill_id) {
  for (auto it = meta_skill_list_.rbegin(); it != meta_skill_list_.rend(); it++) {
    if ((*it)->getMetaSkillId() == meta_skill_id) {
      return *it;
    }
  }
  return nullptr;
}
