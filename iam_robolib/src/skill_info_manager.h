//
// Created by mohit on 11/20/18.
//

#pragma once

#include <vector>

class base_skill;
class base_meta_skill;

class SkillInfoManager {
 public:
  SkillInfoManager();

  base_skill* get_current_skill();

  base_meta_skill* get_current_meta_skill();

  bool is_currently_executing_skill();

  bool is_waiting_for_new_skill();

  void add_skill(base_skill *skill);

  void add_meta_skill(base_meta_skill* skill);

  base_meta_skill* get_meta_skill_with_id(int meta_skill_id);

 private:
  std::vector<base_skill *> skill_list_{};
  std::vector<base_meta_skill *> meta_skill_list_{};
};

