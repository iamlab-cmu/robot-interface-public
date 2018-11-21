//
// Created by mohit on 11/20/18.
//

#pragma once

#include <vector>

#include "task_info.h"

class TaskInfoManager {
    public:
        TaskInfo get_current_task();

        bool is_currently_executing_task();

        bool is_waiting_for_new_task();

        void add_task(TaskInfo task):

    private:
        std::vector<TaskInfo> task_list_;
};

