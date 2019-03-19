//
// Created by jacky on 1/26/19.
//

#include "iam_robolib/skills/force_torque_skill.h"

#include <cassert>
#include <iostream>
#include <vector>
#include <array>

#include <franka/robot.h>
#include <franka/model.h>
#include <franka/exception.h>

#include "iam_robolib/termination_handler/termination_handler.h"
#include "iam_robolib/trajectory_generator/impulse_trajectory_generator.h"
#include "iam_robolib/trajectory_generator/trajectory_generator.h"
#include "iam_robolib/robot_state_data.h"
#include "iam_robolib/run_loop.h"
#include "iam_robolib/run_loop_shared_memory_handler.h"

#include <iam_robolib_common/run_loop_process_info.h>

void ForceTorqueSkill::execute_skill() {
  assert(false);
}

void ForceTorqueSkill::execute_skill_on_franka(run_loop* run_loop,
                                               FrankaRobot* robot,
                                               RobotStateData *robot_state_data) {
  double time = 0.0;
  int log_counter = 0;

  std::cout << "Will run the control loop\n";

  RunLoopSharedMemoryHandler* shared_memory_handler = run_loop->get_shared_memory_handler();
  RunLoopProcessInfo* run_loop_info = shared_memory_handler->getRunLoopProcessInfo();
  boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(
                                  *(shared_memory_handler->getRunLoopProcessInfoMutex()),
                                  boost::interprocess::defer_lock);

  auto force_control_callback = [&](const franka::RobotState& robot_state, 
                    franka::Duration period) -> franka::Torques {
    if (time == 0.0) {
      traj_generator_->initialize_trajectory(robot_state);
    }
    log_counter += 1;
    if (log_counter % 1 == 0) {
      robot_state_data->log_pose_desired(traj_generator_->pose_desired_);
      robot_state_data->log_robot_state(robot_state, time);
    }

    try {
      if (lock.try_lock()) {
        run_loop_info->set_time_since_skill_started(time);
        run_loop_info->set_robot_time(robot_state.time.toSec());
        lock.unlock();
      } 
    } catch (boost::interprocess::lock_exception) {
      // Do nothing
    }

    bool done = termination_handler_->should_terminate_on_franka(robot_state, traj_generator_);
    if (done) {
      // return 0 torques to finish
      std::array<double, 7> tau_d_array{};
      franka::Torques torques(tau_d_array);
      return franka::MotionFinished(torques);
    }
    
    dynamic_cast<ImpulseTrajectoryGenerator*>(traj_generator_)->check_displacement_cap(robot_state);
    time += period.toSec();
    traj_generator_->time_ = time;
    traj_generator_->dt_ = period.toSec();

    traj_generator_->get_next_step();

    feedback_controller_->get_next_step(robot_state, traj_generator_);

    return feedback_controller_->tau_d_array_;
  };

  robot->robot_.control(force_control_callback);
}

