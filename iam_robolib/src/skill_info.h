//
// Created by mohit on 11/20/18.
//

#pragma once

#include "trajectory_generator.h"

enum class SkillStatus { TO_START, RUNNING, FINISHED };  // enum class

class SkillInfo {
  public:
    SkillInfo(int skill_idx): skill_idx_(skill_idx),
                              skill_status_(SkillStatus::TO_START) {};

    int get_current_skill_id();
    void set_skill_status(SkillStatus new_task_status);

    void start_skill(TrajectoryGenerator *traj_generator);

    SkillStatus get_current_skill_status();

    void execute_skill();

  private:
    int skill_idx_;
    SkillStatus skill_status_;

    TrajectoryGenerator *traj_generator_=0;
    // TODO(Mohit): Add Feedback controller and other things here.
};

