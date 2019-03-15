#pragma once

#include <cassert>
#include <string>

class SkillStateInfo {
 public:
  SkillStateInfo(){};

  int get_skill_id();

  void set_skill_id(int skill_id);

  int get_meta_skill_id();

  void set_meta_skill_id(int meta_skill_id);

  int get_skill_type();

  void set_skill_type(int skill_type);

  int get_meta_skill_type();

  void set_meta_skill_type(int meta_skill_type);

  std::string get_skill_description();

  void set_skill_description(std::string skill_description);

  double get_time_since_skill_started();

  void set_time_since_skill_started(double time_since_skill_started);

  bool get_skill_running();

  void set_skill_running(bool skill_running);

  bool get_skill_preempted();

  void set_skill_preempted(bool skill_preempted);

  void reset_skill_info_vars();

 private:
  int skill_id_{-1}; 
  int meta_skill_id_{-1};
  int skill_type_{-1};
  int meta_skill_type_{-1};
  char skill_description_[1000];
  size_t skill_description_len_{0};
  double time_since_skill_started_{0.0};
  bool skill_running_{false};
  bool skill_preempted_{false};
};

