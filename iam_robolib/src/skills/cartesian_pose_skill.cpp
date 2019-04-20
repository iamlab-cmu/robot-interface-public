#include "iam_robolib/skills/cartesian_pose_skill.h"

#include <cassert>
#include <iostream>
#include <array>

#include <franka/robot.h>

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
  double last_period = 0.0;
  double last_2_period = 0.0;
  std::array<double, 16> last_desired_pose; 
  std::array<double, 16> last_2_desired_pose; 
  std::array<double, 16> last_3_desired_pose; 
  double D = 0.5;
  double tau = 0.0;
  std::array<double, 3> initial_position;
  std::array<double, 3> initial_velocity;
  std::array<double, 3> initial_acceleration;
  std::array<double, 3> final_position;
  std::array<double, 3> a_3;
  std::array<double, 3> a_4;
  std::array<double, 3> a_5;
  std::array<double, 3> cur_jerk;
  std::array<double, 3> cur_accel;
  std::array<double, 3> cur_vel;
  std::array<double, 3> cur_pos;
  double velocity_factor = 0.25;
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
      robot_state_data->log_robot_state(desired_pose, robot_state, time);
    } 

    // std::cout << " \n==== Time: " << time << " ==== \n";
    // std::cout << "O_T_EE_d: " << robot_state.O_T_EE_d[12] << ", " << robot_state.O_T_EE_d[13] << ", " << robot_state.O_T_EE_d[14] << std::endl;
    // std::cout << "O_T_EE_c: " << robot_state.O_T_EE_c[12] << ", " << robot_state.O_T_EE_c[13] << ", " << robot_state.O_T_EE_c[14] << std::endl;
    // std::cout << "desired pos: " << desired_pose[12] << ", " << desired_pose[13] << ", " << desired_pose[14] << std::endl;

    // std::cout << "O_force: " << robot_state.O_F_ext_hat_K[0] << ", " << robot_state.O_F_ext_hat_K[1] << ", " << robot_state.O_F_ext_hat_K[2] << std::endl;
    // std::cout << "K_force: " << robot_state.K_F_ext_hat_K[0] << ", " << robot_state.K_F_ext_hat_K[1] << ", " << robot_state.K_F_ext_hat_K[2] << std::endl;
    // std::cout << "commanded accel: " << robot_state.O_ddP_EE_c[0] << ", " << robot_state.O_ddP_EE_c[1] << ", " << robot_state.O_ddP_EE_c[2] << std::endl;
    // std::cout << "commanded vel: " << robot_state.O_dP_EE_c[0] << ", " << robot_state.O_dP_EE_c[1] << ", " << robot_state.O_dP_EE_c[2] << std::endl;
    // std::cout << "commanded pos: " << robot_state.O_T_EE_c[12] << ", " << robot_state.O_T_EE_c[13] << ", " << robot_state.O_T_EE_c[14] << std::endl;

    if((time > 0.0 && done) || skill_termination_handler_end_time > 0.0) {

      std::cout << "\n===== Skill termination end time: " << skill_termination_handler_end_time << " ===== "<<std::endl;

      if (skill_termination_handler_end_time == 0.0) {
        skill_termination_handler_end_time = time;
        // final_O_T_EE = robot_state.O_T_EE_c;
        for(int i = 0; i < 3; i++) {
          // final_O_T_EE[12+i] += 0.001 * robot_state.O_dP_EE_c[i];

          // From http://courses.shadmehrlab.org/Shortcourse/minimumjerk.pdf?fbclid=IwAR1-DaPEDKdYdbrQ5m5Bcm4WfWbVJJT8cLD6XhsRnKY4oPNRmpqoOnEB5os
          //initial_position[i] = robot_state.O_T_EE_c[12+i];
          initial_position[i] = last_desired_pose[12+i];
          initial_velocity[i] = (last_desired_pose[12+i] - last_2_desired_pose[12+i]) / last_period;
          initial_acceleration[i] = (((last_2_desired_pose[12+i] - last_3_desired_pose[12+i]) / last_2_period) - initial_velocity[i]) / last_period;

          cur_accel[i] = initial_acceleration[i];
          cur_vel[i] = initial_velocity[i];
          cur_pos[i] = initial_position[i];

          final_position[i] = robot_state.O_T_EE_c[12+i] + velocity_factor * robot_state.O_dP_EE_c[i];
          a_3[i] = -1.5 * D_pow_2 * initial_acceleration[i] - 6 * D * initial_velocity[i] + 10 * (final_position[i] - initial_position[i]);
          a_4[i] = 1.5 * D_pow_2 * initial_acceleration[i] + 8 * D * initial_velocity[i] - 15 * (final_position[i] - initial_position[i]);
          a_5[i] = -0.5 * D_pow_2 * initial_acceleration[i] - 3 * D * initial_velocity[i] + 6 * (final_position[i] - initial_position[i]);
        }
        
        /*
        std::cout << "============= Forward =================\n";
        for(double tau = 0.0; tau < 0.01; tau += 0.001) {
          for(int i = 0; i < 3; i++) {
            cur_jerk[i] = 6 * a_3[i] / D_pow_3 + 24 * a_4[i] * tau / D_pow_3 + 60 * a_5[i] * pow(tau,2) / D_pow_3;
            cur_accel[i] += cur_jerk[i] * period.toSec();
            cur_vel[i] += cur_accel[i] * period.toSec();
            cur_pos[i] += cur_vel[i] * period.toSec();
          }
          std::cout << "=========== Tau = " << tau << "==============\n";
          std::cout << "desired jerk: " << cur_jerk[0] << ", " << cur_jerk[1] << ", " << cur_jerk[2] << std::endl;
          std::cout << "desired accel: " << cur_accel[0] << ", " << cur_accel[1] << ", " << cur_accel[2] << std::endl;
          std::cout << "desired vel: " << cur_vel[0] << ", " << cur_vel[1] << ", " << cur_vel[2] << std::endl;
          std::cout << "desired pose: " << cur_pos[0] << ", " << cur_pos[1] << ", " << cur_pos[2] << std::endl;
        }

        for(int i = 0; i < 3; i++) {
          cur_accel[i] = initial_acceleration[i];
          cur_vel[i] = initial_velocity[i];
          cur_pos[i] = initial_position[i];
        }
         */
      }

      // if (time - skill_termination_handler_end_time < 0.1) {
      // }

      // if (time - skill_termination_handler_end_time < D) {
      //   D = std::min(std::max((time - skill_termination_handler_end_time) / D, 0.0), 1.0);
      //   slerp_t = (10 * std::pow(D, 3) - 15 * std::pow(D, 4) + 6 * std::pow(D, 5));
  
      //   std::array<double, 6> curr_vel = robot_state.O_dP_EE_c;
      //   std::array<double, 6> next_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

      //   desired_pose = robot_state.O_T_EE_c;

      //   for (int i = 0; i < 3; i++) {
      //     next_vel[i] = initial_vel[i] * (1 - slerp_t);
      //     desired_pose[12 + i] += next_vel[i] * period.toSec();
      //   }

      //   std::cout << "====\n";
      //   std::cout << "commanded accel: " << robot_state.O_ddP_EE_c[0] << ", " << robot_state.O_ddP_EE_c[1] << ", " << robot_state.O_ddP_EE_c[2] << std::endl;
      //   std::cout << "commanded vel: " << robot_state.O_dP_EE_c[0] << ", " << robot_state.O_dP_EE_c[1] << ", " << robot_state.O_dP_EE_c[2] << std::endl;
      //   std::cout << "desired vel: " << robot_state.O_dP_EE_d[0] << ", " << robot_state.O_dP_EE_d[1] << ", " << robot_state.O_dP_EE_d[2] << std::endl;
      //   std::cout << "desired pose: " << desired_pose[12] << ", " << desired_pose[13] << ", " << desired_pose[14] << std::endl;

      //   return desired_pose;
      // }

      if (time - skill_termination_handler_end_time < D) {
        tau = std::min(std::max((time - skill_termination_handler_end_time) / D, 0.0), 1.0);

        desired_pose = robot_state.O_T_EE_c;

        for (int i = 0; i < 3; i++) {
          cur_jerk[i] = 6 * a_3[i] / D_pow_3 + 24 * a_4[i] * tau / D_pow_3 + 60 * a_5[i] * pow(tau,2) / D_pow_3;
          cur_accel[i] += cur_jerk[i] * period.toSec();
          cur_vel[i] += cur_accel[i] * period.toSec();
          cur_pos[i] += cur_vel[i] * period.toSec();

          desired_pose[12 + i] = cur_pos[i];
        }

        // std::cout << "old commanded accel: " << robot_state.O_ddP_EE_c[0] << ", " << robot_state.O_ddP_EE_c[1] << ", " << robot_state.O_ddP_EE_c[2] << std::endl;
        // std::cout << "old desired vel: " << robot_state.O_dP_EE_d[0] << ", " << robot_state.O_dP_EE_d[1] << ", " << robot_state.O_dP_EE_d[2] << std::endl;
        // // std::cout << "new desired accel: " << cur_accel[0] << ", " << cur_accel[1] << ", " << cur_accel[2] << std::endl;
        // std::cout << "new desired vel: " << cur_vel[0] << ", " << cur_vel[1] << ", " << cur_vel[2] << std::endl;
        // // std::cout << "new desired jerk: " << cur_jerk[0] << ", " << cur_jerk[1] << ", " << cur_jerk[2] << std::endl;
        // std::cout << "new desired pose: " << desired_pose[12] << ", " << desired_pose[13] << ", " << desired_pose[14] << std::endl;
        // std::cout << "====\n\n";

        return desired_pose;
      }

      // if (time - skill_termination_handler_end_time < D) {
      //   D = std::min(std::max((time - skill_termination_handler_end_time) / D, 0.0), 1.0);
      //   slerp_t = (10 * std::pow(D, 3) - 15 * std::pow(D, 4) + 6 * std::pow(D, 5));
  
      //   std::array<double, 6> curr_vel = robot_state.O_dP_EE_c;
      //   std::array<double, 6> next_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

      //   desired_pose = robot_state.O_T_EE_c;

      //   for (int i = 0; i < 3; i++) {
      //     next_vel[i] = initial_vel[i] * (1 - slerp_t);
      //     desired_pose[12 + i] += next_vel[i] * period.toSec();
      //   }

      //   std::cout << "====\n";
      //   std::cout << "commanded accel: " << robot_state.O_ddP_EE_c[0] << ", " << robot_state.O_ddP_EE_c[1] << ", " << robot_state.O_ddP_EE_c[2] << std::endl;
      //   std::cout << "commanded vel: " << robot_state.O_dP_EE_c[0] << ", " << robot_state.O_dP_EE_c[1] << ", " << robot_state.O_dP_EE_c[2] << std::endl;
      //   std::cout << "desired vel: " << robot_state.O_dP_EE_d[0] << ", " << robot_state.O_dP_EE_d[1] << ", " << robot_state.O_dP_EE_d[2] << std::endl;
      //   std::cout << "desired pose: " << desired_pose[12] << ", " << desired_pose[13] << ", " << desired_pose[14] << std::endl;

      //   return desired_pose;
      // }
     
      // if (time - skill_termination_handler_end_time < 0.1) {
      //   std::cout << "Here return desired O_T_EE_D" << std::endl;
      //   std::cout << "commanded vel: " << robot_state.O_dP_EE_c[0] << ", " << robot_state.O_dP_EE_c[1] << ", " << robot_state.O_dP_EE_c[2] << std::endl;
      //   std::cout << "desired vel: " << robot_state.O_dP_EE_d[0] << ", " << robot_state.O_dP_EE_d[1] << ", " << robot_state.O_dP_EE_d[2] << std::endl;
      //   std::cout << "commanded accel: " << robot_state.O_ddP_EE_c[0] << ", " << robot_state.O_ddP_EE_c[1] << ", " << robot_state.O_ddP_EE_c[2] << std::endl;

      //   std::array<double, 6> curr_vel = robot_state.O_dP_EE_c;
      //   std::array<double, 6> curr_accel = robot_state.O_ddP_EE_c;

      //   std::array<double, 6> max_deaccel = {10.0, 10.0, 10.0, 24.0, 24.0, 24.0};
      //   std::array<double, 6> next_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        
      //   desired_pose = robot_state.O_T_EE_c;
        
      //   for (size_t i = 0; i < curr_vel.size(); i++) {
      //     // next_vel[i] =  curr_vel[i] + (curr_accel[i] * 0.9) * period.toSec();
      //     if (curr_accel[i] > 0.001)  {
      //       next_vel[i] =  curr_vel[i] + 0.001 * period.toSec();
      //     } else {
      //        next_vel[i] =  curr_vel[i] + (curr_accel[i] * 0.9) * period.toSec();
      //           }

          
      //     if (curr_vel[i] > 0.0) {
      //             next_vel[i] = std::max(1e-12, curr_vel[i] - max_deaccel[i] * period.toSec());
      //     } else {
      //       next_vel[i] = std::min(-1e-12, curr_vel[i] + max_deaccel[i] * period.toSec());
      //     }
          
      //     if (i < 3) {
      //        desired_pose[12 + i] += next_vel[i] * period.toSec();
      //     }
      //   }
        
      //   std::cout << "vel: " << next_vel[0] << ", " << next_vel[1] << ", " << next_vel[2] << std::endl;
      //   std::cout << "desired_pose: " << desired_pose[12] << ", " << desired_pose[13] << ", " << desired_pose[14] << std::endl;

      //   // return O_T_EE_c;
      //   return desired_pose;
      // }

  
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
    last_3_desired_pose = last_2_desired_pose;
    last_2_desired_pose = last_desired_pose;
    last_desired_pose = desired_pose;
    last_2_period = last_period;
    last_period = period.toSec();
    return desired_pose;
  };

  robot->robot_.control(cartesian_pose_callback, franka::ControllerMode::kCartesianImpedance);
}

