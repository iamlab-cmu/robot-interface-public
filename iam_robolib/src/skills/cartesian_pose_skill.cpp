#include "iam_robolib/skills/cartesian_pose_skill.h"

#include <cassert>
#include <iostream>
#include <array>

#include <franka/robot.h>

#include <boost/circular_buffer.hpp>

#include "iam_robolib/robot_state_data.h"
#include "iam_robolib/run_loop.h"
#include "iam_robolib/run_loop_shared_memory_handler.h"
#include "iam_robolib/trajectory_generator/pose_trajectory_generator.h"

#include <iam_robolib_common/definitions.h>
#include <iam_robolib_common/run_loop_process_info.h>

void CartesianPoseSkill::execute_skill() { assert(false); 
} 
void CartesianPoseSkill::execute_skill_on_franka(run_loop* run_loop,
                                                 FrankaRobot* robot,
                                                 RobotStateData *robot_state_data) {
  double time = 0.0;
  double skill_termination_handler_end_time = 0.0;
  int log_counter = 0;

  // Circular buffers for smooth deacceleration after stopping.
  boost::circular_buffer<double> last_periods_cb(3);
  boost::circular_buffer<std::array<double, 16>> last_pose_cb(3);

  // Time for smooth deacceleration after abrupt stopping after feeling contact.
  // TODO: Can we reduce this time further?
  double D = 0.1;
  double extra_time_factor = 1.3;
  double max_accel = 3.0;
  double eps = 0.001;

  std::array<double, 3> cur_jerk;
  std::array<double, 3> cur_accel;
  std::array<double, 3> cur_vel;
  std::array<double, 3> cur_pos;

  std::array<double, 3> initial_position;
  std::array<double, 3> initial_velocity;
  std::array<double, 3> initial_acceleration;
  std::array<double, 3> final_position;
  std::array<double, 3> a_3;
  std::array<double, 3> a_4;
  std::array<double, 3> a_5;
  double D_pow_2 = pow(D,2);
  double D_pow_3 = pow(D,3);

  RunLoopSharedMemoryHandler* shared_memory_handler = run_loop->get_shared_memory_handler();
  RunLoopProcessInfo* run_loop_info = shared_memory_handler->getRunLoopProcessInfo();
  boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(
                                  *(shared_memory_handler->getRunLoopProcessInfoMutex()),
                                  boost::interprocess::defer_lock);

  PoseTrajectoryGenerator* pose_trajectory_generator = dynamic_cast<PoseTrajectoryGenerator*>(traj_generator_);

  if(pose_trajectory_generator == nullptr) {
    throw std::bad_cast();
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
    if (time > 0.0) {
      traj_generator_->get_next_step();
    }

    bool done = termination_handler_->should_terminate_on_franka(robot_state, traj_generator_);

    std::array<double, 16> desired_pose = pose_trajectory_generator->get_desired_pose();

    log_counter += 1;
    if (log_counter % 1 == 0) {
      robot_state_data->log_robot_state(desired_pose, robot_state, robot->getModel(), time);
    } 

    if((time > 0.0 && done) || skill_termination_handler_end_time > 0.0) {
      double last_period = last_periods_cb[2];
      double second_last_period = last_periods_cb[1];

      if (skill_termination_handler_end_time == 0.0) {
        skill_termination_handler_end_time = time;
        for(int i = 0; i < 3; i++) {
          // From http://courses.shadmehrlab.org/Shortcourse/minimumjerk.pdf?fbclid=IwAR1-DaPEDKdYdbrQ5m5Bcm4WfWbVJJT8cLD6XhsRnKY4oPNRmpqoOnEB5os
          initial_position[i] = last_pose_cb[2][12+i];
          initial_velocity[i] = (last_pose_cb[2][12+i] - last_pose_cb[1][12+i]) / last_period;
          initial_acceleration[i] = (((last_pose_cb[1][12+i] - last_pose_cb[0][12+i]) / second_last_period) - initial_velocity[i]) / last_period;

          cur_accel[i] = initial_acceleration[i];
          cur_vel[i] = initial_velocity[i];
          cur_pos[i] = initial_position[i];

          D = std::max(D, abs(cur_vel[i] / max_accel));
        }

        D *= extra_time_factor;

        D_pow_2 = pow(D,2);
        D_pow_3 = pow(D,3);

        for(int i = 0; i < 3; i++) {
          if(abs(cur_vel[i]) < eps) {
            final_position[i] = initial_position[i] + initial_velocity[i] * D;
          }
          else if (cur_vel[i] > 0) {
            final_position[i] = initial_position[i] + pow(initial_velocity[i],2) * extra_time_factor / max_accel;
          } else {
            final_position[i] = initial_position[i] - pow(initial_velocity[i],2) * extra_time_factor / max_accel;
          }
        }

        for(int i = 0; i < 3; i++) {
          a_3[i] = -1.5 * D_pow_2 * initial_acceleration[i] - 6 * D * initial_velocity[i] + 10 * (final_position[i] - initial_position[i]);
          a_4[i] = 1.5 * D_pow_2 * initial_acceleration[i] + 8 * D * initial_velocity[i] - 15 * (final_position[i] - initial_position[i]);
          a_5[i] = -0.5 * D_pow_2 * initial_acceleration[i] - 3 * D * initial_velocity[i] + 6 * (final_position[i] - initial_position[i]);
        }
      }

      if (time - skill_termination_handler_end_time < D) {
        double tau = std::min(std::max((time - skill_termination_handler_end_time) / D, 0.0), 1.0);

        desired_pose = robot_state.O_T_EE_c;

        for (int i = 0; i < 3; i++) {
          cur_jerk[i] = 6 * a_3[i] / D_pow_3 + 24 * a_4[i] * tau / D_pow_3 + 60 * a_5[i] * pow(tau,2) / D_pow_3;
          cur_accel[i] += cur_jerk[i] * period.toSec();
          cur_vel[i] += cur_accel[i] * period.toSec();
          cur_pos[i] += cur_vel[i] * period.toSec();

          desired_pose[12 + i] = cur_pos[i];
        }

        return desired_pose;
      }

  
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
    // Add items to circular buffer.
    last_pose_cb.push_back(desired_pose);
    last_periods_cb.push_back(period.toSec());
    // Just fill the buffers initially with whatever we currently have. Prevents the edge case of stopping
    // right when we begin.
    while (!last_pose_cb.full()) {
      last_pose_cb.push_back(desired_pose);
    }
    while (!last_periods_cb.full()) {
      last_periods_cb.push_back(0.001);
    }
    return desired_pose;
  };

  robot->robot_.control(cartesian_pose_callback, franka::ControllerMode::kCartesianImpedance);
}

