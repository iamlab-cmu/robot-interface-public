//
// Created by mohit on 11/20/18.
//

#include "skill_info.h"

int SkillInfo::get_current_task_id() {
    return task_idx_;
}

SkillStatus SkillInfo::get_current_task_status() {
    return task_status_;
}

void SkillInfo::set_task_status(SkillStatus new_task_status) {
    // TODO(Mohit): Maybe add checks such that task status progresses
    // in one direction.
    task_status_ = new_task_status;
}