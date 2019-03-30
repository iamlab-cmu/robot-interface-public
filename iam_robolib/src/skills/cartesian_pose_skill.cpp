#include "iam_robolib/skills/cartesian_pose_skill.h"

#include <cassert>
#include <iostream>
#include <vector>
#include <array>

#include <franka/robot.h>
#include <franka/exception.h>

#include "iam_robolib/run_loop.h"
#include "iam_robolib/robot_state_data.h"
#include "iam_robolib/trajectory_generator/pose_trajectory_generator.h"
#include "iam_robolib/run_loop_shared_memory_handler.h"

#include <iam_robolib_common/definitions.h>
#include <iam_robolib_common/run_loop_process_info.h>

void CartesianPoseSkill::execute_skill() {
  assert(false);
}

void CartesianPoseSkill::execute_skill_on_franka(run_loop* run_loop,
                                                 FrankaRobot* robot,
                                                 RobotStateData *robot_state_data) {
  double time = 0.0;
  int log_counter = 0;

  RunLoopSharedMemoryHandler* shared_memory_handler = run_loop->get_shared_memory_handler();
  RunLoopProcessInfo* run_loop_info = shared_memory_handler->getRunLoopProcessInfo();
  boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(
                                  *(shared_memory_handler->getRunLoopProcessInfoMutex()),
                                  boost::interprocess::defer_lock);

  PoseTrajectoryGenerator* pose_trajectory_generator = dynamic_cast<PoseTrajectoryGenerator*>(traj_generator_);

  if(pose_trajectory_generator == nullptr) {
    throw 333;
  }

  std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)>
      cartesian_pose_callback = [&](
      const franka::RobotState& robot_state,
      franka::Duration period) -> franka::CartesianPose {

    if (time == 0.0) {
      pose_trajectory_generator->initialize_trajectory(robot_state, SkillType::CartesianPoseSkill);
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

    log_counter += 1;
    if (log_counter % 1 == 0) {
      robot_state_data->log_robot_state(robot_state, time);
    }

    std::array<double, 16> desired_pose = pose_trajectory_generator->get_desired_pose();

    if(done) {
      try {
        if (lock.try_lock()) {
          run_loop_info->set_time_skill_finished_in_robot_time(robot_state.time.toSec());
          lock.unlock();
        } 
      } catch (boost::interprocess::lock_exception) {
        // Do nothing
      }
      return franka::MotionFinished(desired_pose);
    }

    return desired_pose;
  };

  robot->robot_.control(cartesian_pose_callback);
}

