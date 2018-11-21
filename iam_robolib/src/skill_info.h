//
// Created by mohit on 11/20/18.
//

#pragma once

enum class SkillStatus { TO_START, RUNNING, FINISHED };  // enum class

class SkillInfo {
  public:
    SkillInfo(int task_idx): task_idx_(task_idx) {};

    int get_current_task_id();

    void set_task_status(SkillStatus new_task_status);
    SkillStatus get_current_task_status();


  private:
    int task_idx_;
    SkillStatus task_status_{SkillStatus::TO_START};
};

