//
// Created by mohit on 11/20/18.
//

#pragma once

#include <vector>

#include "skill_info.h"

class TaskInfoManager {
    public:
        SkillInfo get_current_task();

        bool is_currently_executing_task();

        bool is_waiting_for_new_task();

        void add_task(SkillInfo task):

    private:
        std::vector<SkillInfo> task_list_;
};

