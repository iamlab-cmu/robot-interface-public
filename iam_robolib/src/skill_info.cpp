//
// Created by mohit on 11/20/18.
//

#include "skill_info.h"

int SkillInfo::get_current_skill_id(){
    return skill_idx_;
}

SkillStatus SkillInfo::get_current_skill_status() {
    return skill_status_;
}

void SkillInfo::set_skill_status(SkillStatus new_task_status){
    // TODO(Mohit): Maybe add checks such that task status progresses
    // in one direction.
    skill_status_ = new_task_status;
}