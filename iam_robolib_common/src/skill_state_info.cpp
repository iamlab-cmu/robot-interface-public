#include <iam_robolib_common/skill_state_info.h>

#include <cstring>

int SkillStateInfo::get_skill_id() {
  return skill_id_;
}

void SkillStateInfo::set_skill_id(int skill_id) {
  skill_id_ = skill_id;
}

int SkillStateInfo::get_meta_skill_id() {
  return meta_skill_id_;
}

void SkillStateInfo::set_meta_skill_id(int meta_skill_id) {
  meta_skill_id_ = meta_skill_id;
}

int SkillStateInfo::get_skill_type() {
  return skill_type_;
}

void SkillStateInfo::set_skill_type(int skill_type) {
  skill_type_ = skill_type;
}

int SkillStateInfo::get_meta_skill_type() {
  return meta_skill_type_;
}

void SkillStateInfo::set_meta_skill_type(int meta_skill_type) {
  meta_skill_type_ = meta_skill_type;
}

std::string SkillStateInfo::get_skill_description() {
  std::string skill_description(skill_description_, skill_description_ + 
                    sizeof(skill_description_[0]) * skill_description_len_);
  return skill_description;
}

void SkillStateInfo::set_skill_description(std::string skill_description) {
  std::memcpy(&skill_description_, skill_description.c_str(), skill_description.size());
  skill_description_len_  = skill_description.size();
}

double SkillStateInfo::get_time_since_skill_started() {
  return time_since_skill_started_;
}

void SkillStateInfo::set_time_since_skill_started(double time_since_skill_started) {
  time_since_skill_started_ = time_since_skill_started;
}

bool SkillStateInfo::get_skill_running() {
  return skill_running_;
}

void SkillStateInfo::set_skill_running(bool skill_running) {
  skill_running = skill_running_;
}

bool SkillStateInfo::get_skill_preempted() {
  return skill_preempted_;
}

void SkillStateInfo::set_skill_preempted(bool skill_preempted) {
  skill_preempted_ = skill_preempted;
}

void SkillStateInfo::reset_skill_info_vars() {
  skill_id_ = -1; 
  meta_skill_id_ = -1;
  skill_type_ = -1;
  meta_skill_type_ = -1;
  memset(skill_description_, 0, sizeof skill_description_);
  skill_description_len_ = 0;
  time_since_skill_started_ = 0.0;
  skill_running_ = false;
  skill_preempted_ = false;
}