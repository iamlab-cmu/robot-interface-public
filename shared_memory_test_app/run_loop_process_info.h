//
// Created by mohit on 11/20/18.
//

#pragma once

#include <string>

class RunLoopProcessInfo {
    public:
        RunLoopProcessInfo(int memory_region_idx): current_memory_region_(memory_region_idx) {};

        bool new_task_available_{false};

        std::string get_current_shared_memory_name();
        int get_current_shared_memory_index();

    private:
        int current_memory_region_;
        int current_skill_id = -1;
};
