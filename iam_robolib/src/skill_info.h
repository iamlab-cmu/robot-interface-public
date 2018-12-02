//
// Created by mohit on 11/20/18.
//

#pragma once

#include "BaseSkill.h"

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include "BaseSkill.h"
#include "FeedbackController.h"
#include "TerminationHandler.h"
#include "trajectory_generator.h"

class SkillInfo : public BaseSkill {
  public:
    SkillInfo(int skill_idx): BaseSkill(skill_idx) {};


    virtual void execute_skill() override;

    virtual void execute_skill_on_franka(franka::Robot *robot, franka::Gripper *gripper,
                                         ControlLoopData *control_loop_data) override;

    virtual void execute_skill_on_franka_temp(franka::Robot *robot, franka::Gripper *gripper,
                                              ControlLoopData *control_loop_data) override;
    
    virtual void execute_skill_on_franka_temp2(franka::Robot *robot, franka::Gripper *gripper,
                                               ControlLoopData *control_loop_data) override;

    virtual bool should_terminate();

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

 protected:
    const std::array<double, 7> k_gains_ = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
    // Damping
    const std::array<double, 7> d_gains_ = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};
};

