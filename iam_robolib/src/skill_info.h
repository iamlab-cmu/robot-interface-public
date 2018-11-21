//
// Created by mohit on 11/20/18.
//

#pragma once

enum class SkillStatus { TO_START, RUNNING, FINISHED };  // enum class

class SkillInfo {
  public:
    SkillInfo(int skill_idx): skill_idx_(skill_idx) {};

    int get_current_skill_id();

    void set_skill_status(SkillStatus new_task_status);
    SkillStatus get_current_skill_status();


  private:
    int skill_idx_;
    SkillStatus skill_status_{SkillStatus::TO_START};
};

