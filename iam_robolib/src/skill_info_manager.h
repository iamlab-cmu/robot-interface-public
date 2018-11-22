//
// Created by mohit on 11/20/18.
//

#pragma once

#include <vector>

#include "skill_info.h"

class SkillInfoManager {
 public:
  SkillInfoManager();

  SkillInfo *get_current_skill();

  bool is_currently_executing_skill();

  bool is_waiting_for_new_skill();

  void add_skill(SkillInfo *skill);

 private:
  std::vector<SkillInfo *> skill_list_{};
};

