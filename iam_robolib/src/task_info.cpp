//
// Created by mohit on 11/20/18.
//

#include "task_info.h"

int TaskInfo::get_current_task_id() {
    return task_idx_;
}

TaskStatus TaskInfo::get_current_task_status() {
    return task_status_;
}

void TaskInfo::set_task_status(TaskStatus new_task_status) {
    // TODO(Mohit): Maybe add checks such that task status progresses
    // in one direction.
    task_status_ = new_task_status;
}