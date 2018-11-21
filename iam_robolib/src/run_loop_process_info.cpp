//
// Created by mohit on 11/20/18.
//

#include "run_loop_process_info.h"


int RunLoopProcessInfo::get_current_shared_memory_index() {
    return current_memory_region_;
}

std::string RunLoopProcessInfo::get_current_shared_memory_name() {
    return "run_loop_shared_memory_" + current_memory_region_;
}