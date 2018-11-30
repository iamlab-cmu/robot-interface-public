//
// Created by mohit on 11/20/18.
//

#include "skill_info.h"

#include <cassert>
#include <iostream>
#include <vector>
#include <array>

#include <franka/exception.h>

#include "ControlLoopData.h"

int SkillInfo::get_skill_id() {
    return skill_idx_;
}

void SkillInfo::set_skill_status(SkillStatus status) {
  // TODO(Mohit): Maybe add checks such that task status progresses
  // in one direction.
  skill_status_ = status;
}

void SkillInfo::start_skill(TrajectoryGenerator *traj_generator,
                            FeedbackController *feedback_controller,
                            TerminationHandler *termination_handler) {
  skill_status_ = SkillStatus::TO_START;
  traj_generator_ = traj_generator;
  traj_generator_->initialize_trajectory();
  feedback_controller_ = feedback_controller;
  feedback_controller_->initialize_controller();
  termination_handler_ = termination_handler;
  termination_handler_->initialize_handler();
}

SkillStatus SkillInfo::get_current_skill_status() {
    return skill_status_;
}

void SkillInfo::execute_skill() {
  assert(traj_generator_ != 0);
  // HACK
  std::string skill_status_string = "Running";
  if (skill_status_ == SkillStatus::FINISHED) {
    skill_status_string = "Finished";
  }
  std::cout << "Will execute skill with status: " << skill_status_string << "\n";
  traj_generator_->get_next_step();
}

void SkillInfo::execute_skill_on_franka(franka::Robot* robot, ControlLoopData *control_loop_data) {

  std::vector<std::array<double, 16>> log_pose_desired{};
  std::vector<std::array<double, 16>> log_robot_state{};
  std::vector<std::array<double, 7>> log_tau_j;
  std::vector<std::array<double, 7>> log_dq;
  std::vector<double> log_control_time;
try { 
  double time = 0.0;
  int log_counter = 0;


  std::cout << "Will run the control loop\n";
  std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)> cartesian_pose_callback =
                                  [=, &time, &log_counter, &log_pose_desired, &log_robot_state, &log_control_time, &log_tau_j, &log_dq](const franka::RobotState&
                                  robot_state,
                                             franka::Duration period) -> franka::CartesianPose {
    if (time == 0.0) {
      traj_generator_->initialize_trajectory(robot_state);
      traj_generator_->dt_ = period.toSec();
    }

    if (control_loop_data->mutex_.try_lock()) {
      control_loop_data->counter_ += 1;
      control_loop_data->time_ =  period.toSec();
      control_loop_data->has_data_ = true;
      control_loop_data->mutex_.unlock();
    }
    traj_generator_->time_ += period.toSec();
    time += period.toSec();
    log_counter += 1;

    traj_generator_->get_next_step();

    bool done = termination_handler_->should_terminate(traj_generator_);

    franka::CartesianPose pose_desired(traj_generator_->pose_desired_);
    if (log_counter % 1 == 0) {
      log_pose_desired.push_back(traj_generator_->pose_desired_);
      log_robot_state.push_back(robot_state.O_T_EE_c);
      log_tau_j.push_back(robot_state.tau_J);
      log_dq.push_back(robot_state.dq);
      log_control_time.push_back(time);
    }

    if(done or time >= traj_generator_->run_time_ + traj_generator_->acceleration_time_)
    {
      return franka::MotionFinished(pose_desired);
    }

    return pose_desired;
  };

    // robot.control([&initial_position, &time](const franka::RobotState& robot_state,
    //                                          franka::Duration period) -> franka::JointPositions {
    //   time += period.toSec();

    //   if (time == 0.0) {
    //     initial_position = robot_state.q_d;
    //   }

    //   double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * time));

    //   franka::JointPositions output = {{initial_position[0], initial_position[1],
    //                                     initial_position[2], initial_position[3] + delta_angle,
    //                                     initial_position[4] + delta_angle, initial_position[5],
    //                                     initial_position[6] + delta_angle}};

    //   if (time >= 5.0) {
    //     std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
    //     return franka::MotionFinished(output);
    //   }
    //   return output;
    // });


  franka::Model model = robot->loadModel();

  std::function<franka::Torques(const franka::RobotState&, franka::Duration)> impedance_control_callback =
            [=, &model](
                const franka::RobotState& state, franka::Duration /*period*/) -> franka::Torques {
      // Read current coriolis terms from model.
      std::array<double, 7> coriolis = model.coriolis(state);

      // Compute torque command from joint impedance control law.
      // Note: The answer to our Cartesian pose inverse kinematics is always in state.q_d with one
      // time step delay.
      std::array<double, 7> tau_d_calculated;
      for (size_t i = 0; i < 7; i++) {
        tau_d_calculated[i] =
            k_gains_[i] * (state.q_d[i] - state.q[i]) - d_gains_[i] * state.dq[i] + coriolis[i];
      }

      // The following line is only necessary for printing the rate limited torque. As we activated
      // rate limiting for the control loop (activated by default), the torque would anyway be
      // adjusted!
      std::array<double, 7> tau_d_rate_limited =
          franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);

      // Send torque command.
      return tau_d_rate_limited;
    };

  /*int memory_index = run_loop_info_->get_current_shared_memory_index();
  SharedBuffer buffer = execution_feedback_buffer_0_;
  if (memory_index == 1) {
    buffer = execution_feedback_buffer_1_;
  }
  skill->write_feedback_to_shared_memory(buffer);

  robot.control()*/

  // robot->control(impedance_control_callback, cartesian_pose_callback);
  robot->control(cartesian_pose_callback, franka::ControllerMode::kCartesianImpedance, true, 1000.0);
} catch (const franka::Exception& ex) {
    std::cerr << ex.what() << std::endl;
    
    std::cout << "===== Robots state ======\n";
    int print_last = 50;
    int print_counter = 1;
    for (auto it = log_robot_state.rbegin(); it != log_robot_state.rend(); it++, print_counter++) {
      std::array<double, 16> temp = *it;
      std::cout << "Time: " << log_control_time[log_robot_state.size() - print_counter] << ",   ";
      for (size_t j = 0; j < temp.size(); j++) {
        std::cout << temp[j] << " ";  
      }
      std::cout << "\n" << std::endl;
      print_last = print_last - 1;
      if (print_last < 0) {
        break;
      }
    }

    std::cout << "===== Desired Pose ======\n";

    print_last = 50;
    print_counter = 1;
    for (auto it = log_pose_desired.rbegin(); it != log_pose_desired.rend(); it++, print_counter++) {
      std::cout << "Time: " << log_control_time[log_robot_state.size() - print_counter] << ",   ";
      std::array<double, 16> temp = *it;

      for (size_t j = 0; j < temp.size(); j++) {
        std::cout << temp[j] << " ";  
      }
      std::cout << "\n" << std::endl;
      print_last = print_last - 1;
      if (print_last < 0) {
        break;
      }
    }

    std::cout << "===== Measured link-side joint torque sensor signals ======\n";

    print_last = 50;
    print_counter = 1;
    for (auto it = log_tau_j.rbegin(); it != log_tau_j.rend(); it++, print_counter++) {
      std::cout << "Time: " << log_control_time[log_robot_state.size() - print_counter] << ",   ";
      std::array<double, 7> temp = *it;

      for (size_t j = 0; j < temp.size(); j++) {
        std::cout << temp[j] << " ";  
      }
      std::cout << "\n" << std::endl;
      print_last = print_last - 1;
      if (print_last < 0) {
        break;
      }
    }

    std::cout << "===== Measured joint velocity ======\n";

    print_last = 50;
    print_counter = 1;
    for (auto it = log_dq.rbegin(); it != log_dq.rend(); it++, print_counter++) {
      std::cout << "Time: " << log_control_time[log_robot_state.size() - print_counter] << ",   ";
      std::array<double, 7> temp = *it;

      for (size_t j = 0; j < temp.size(); j++) {
        std::cout << temp[j] << " ";  
      }
      std::cout << "\n" << std::endl;
      print_last = print_last - 1;
      if (print_last < 0) {
        break;
      }
    }

    std::cout << "===== Measured joint jerks ======\n";

    print_last = 50;
    print_counter = 1;
    for (auto it = log_dq.rbegin(); it != log_dq.rend(); it++, print_counter++) {
      std::cout << "Time: " << log_control_time[log_robot_state.size() - print_counter] << ",   ";
      std::array<double, 7> temp = *it;
      std::array<double, 7> temp2 = *(it+1);
      std::array<double, 7> temp3 = *(it+2);

      for (size_t j = 0; j < temp.size(); j++) {
        std::cout << (((temp3[j] - temp2[j]) * 1000) - ((temp2[j] - temp[j]) * 1000)) * 1000  << " ";  
      }
      std::cout << "\n" << std::endl;
      print_last = print_last - 1;
      if (print_last < 0) {
        break;
      }
    }

}
}

bool SkillInfo::should_terminate() {
  return termination_handler_->should_terminate(traj_generator_);
}

void SkillInfo::write_result_to_shared_memory(float *result_buffer) {
  std::cout << "Should write result to shared memory\n";
}

void SkillInfo::write_feedback_to_shared_memory(float *feedback_buffer) {
  std::cout << "Should write feedback to shared memory\n";
}

