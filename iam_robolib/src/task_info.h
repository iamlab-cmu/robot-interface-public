//
// Created by mohit on 11/20/18.
//

#pragma once

enum class TaskStatus { TO_START, RUNNING, FINISHED };  // enum class

class TaskInfo {
  public:
    TaskInfo(int task_idx): task_idx_(task_idx) {};

    int get_current_task_id();

    void set_task_status(TaskStatus new_task_status);
    TaskStatus get_current_task_status();


  private:
    int task_idx_;
    TaskStatus task_status_{TaskStatus::TO_START};
};

