//
// Created by mohit on 11/20/18.
//

#ifndef IAM_ROBOLIB_RUN_LOOP_PROCESS_INFO_H
#define IAM_ROBOLIB_RUN_LOOP_PROCESS_INFO_H

#include <string>

class RunLoopProcessInfo {
    public:
        RunLoopProcessInfo(int memory_region_idx): current_memory_region_(memory_region_idx) {};

        std::string  get_current_shared_memory_name();
        int get_current_shared_memory_index();

    private:
        int current_memory_region_{0};
};


#endif // IAM_ROBOLIB_RUN_LOOP_PROCESS_INFO_H
