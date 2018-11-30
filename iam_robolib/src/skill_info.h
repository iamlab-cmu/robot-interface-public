//
// Created by mohit on 11/20/18.
//

#pragma once

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include "FeedbackController.h"
#include "TerminationHandler.h"
#include "trajectory_generator.h"

class RunLoop;

enum class SkillStatus { TO_START, RUNNING, FINISHED };  // enum class

class SkillInfo {
  public:
    SkillInfo(int skill_idx): skill_idx_(skill_idx),
                              skill_status_(SkillStatus::TO_START),
                              robot_("172.16.0.2") {};

    int get_skill_id();

    void set_skill_status(SkillStatus new_task_status);

    void start_skill(TrajectoryGenerator *traj_generator,
                     FeedbackController *feedback_controller,
                     TerminationHandler *termination_handler);

    SkillStatus get_current_skill_status();

    void execute_skill();

    void execute_skill_on_franka(RunLoop *run_loop);

    bool should_terminate();

    /**
     * Write result to the shared memory after skill is done.
     * @param result_buffer
     */
    void write_result_to_shared_memory(float *result_buffer);

    /**
     * Write feedback result to the shared memory as feedback for a skill.
     * @param feedback_buffer
     */
    void write_feedback_to_shared_memory(float *feedback_buffer);

  private:
    int skill_idx_;
    SkillStatus skill_status_;

    franka::Robot robot_;

    TrajectoryGenerator *traj_generator_= nullptr;
    FeedbackController *feedback_controller_= nullptr;
    TerminationHandler *termination_handler_= nullptr;

    const std::array<double, 7> k_gains_ = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
    // Damping
    const std::array<double, 7> d_gains_ = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};
};

