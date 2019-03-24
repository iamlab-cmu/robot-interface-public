#include "iam_robolib/skills/joint_pose_with_torque_control_skill.h"

#include <cassert>
#include <iostream>
#include <vector>
#include <array>

#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/rate_limiting.h>

#include "iam_robolib/run_loop.h"
#include "iam_robolib/robot_state_data.h"
#include "iam_robolib/feedback_controller/feedback_controller.h"
#include "iam_robolib/termination_handler/termination_handler.h"
#include "iam_robolib/trajectory_generator/trajectory_generator.h"
#include "iam_robolib/run_loop.h"
#include "iam_robolib/run_loop_shared_memory_handler.h"

#include <iam_robolib_common/run_loop_process_info.h>

void JointPoseWithTorqueControlSkill::execute_skill() {
  assert(false);
}

void JointPoseWithTorqueControlSkill::execute_skill_on_franka(run_loop* run_loop,
                                                              FrankaRobot* robot,
                                                              RobotStateData *robot_state_data) {
  double time = 0.0;
  int log_counter = 0;

  RunLoopSharedMemoryHandler* shared_memory_handler = run_loop->get_shared_memory_handler();
  RunLoopProcessInfo* run_loop_info = shared_memory_handler->getRunLoopProcessInfo();
  boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(
                                  *(shared_memory_handler->getRunLoopProcessInfoMutex()),
                                  boost::interprocess::defer_lock);

  std::cout << "Will run the control loop\n";

  std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)>
      joint_pose_callback = [&](
      const franka::RobotState& robot_state,
      franka::Duration period) -> franka::JointPositions {
    if (time == 0.0) {
      traj_generator_->initialize_trajectory(robot_state);
      try {
        if (lock.try_lock()) {
          run_loop_info->set_time_skill_started_in_robot_time(robot_state.time.toSec());
          lock.unlock();
        } 
      } catch (boost::interprocess::lock_exception) {
        // Do nothing
      }
    }
    time += period.toSec();
    traj_generator_->time_ = time;
    traj_generator_->dt_ = period.toSec();
    traj_generator_->get_next_step();

    bool done = termination_handler_->should_terminate_on_franka(robot_state, traj_generator_);
    franka::JointPositions joint_desired(traj_generator_->joint_desired_);

    log_counter += 1;
    if (log_counter % 1 == 0) {
      robot_state_data->log_pose_desired(traj_generator_->pose_desired_);
      robot_state_data->log_robot_state(robot_state, time);
    }

    if(done) {
      try {
        if (lock.try_lock()) {
          run_loop_info->set_time_skill_finished_in_robot_time(robot_state.time.toSec());
          lock.unlock();
        } 
      } catch (boost::interprocess::lock_exception) {
        // Do nothing
      }

      return franka::MotionFinished(joint_desired);
    }
    return joint_desired;
  };

  std::function<franka::Torques(const franka::RobotState&,
      franka::Duration)> impedance_control_callback = [&](
        const franka::RobotState& state, franka::Duration) -> franka::Torques {
        feedback_controller_->get_next_step();
        return feedback_controller_->tau_d_array_;
      };

  robot->robot_.control(impedance_control_callback, joint_pose_callback);
}

