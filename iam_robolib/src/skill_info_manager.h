//
// Created by mohit on 11/20/18.
//

#pragma once

#include <vector>

class BaseSkill;

class SkillInfoManager {
 public:
  SkillInfoManager();

  BaseSkill *get_current_skill();

  bool is_currently_executing_skill();

  bool is_waiting_for_new_skill();

  void add_skill(BaseSkill *skill);

 private:
  std::vector<BaseSkill *> skill_list_{};
};

