//
// Created by mohit on 11/20/18.
//

#include "skill_info.h"

#include <cassert>
#include <iostream>

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

void SkillInfo::execute_skill_on_franka(franka::Robot* robot) {

  double time = 0.0;

  std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)> cartesian_pose_callback = 
                                  [=, &time](
                                                   const franka::RobotState& robot_state,
                                                   franka::Duration period) -> franka::CartesianPose {

    time += period.toSec();

    traj_generator_->get_next_step();

    bool done = termination_handler_->should_terminate(traj_generator_);

    franka::CartesianPose pose_desired = traj_generator_->pose_desired_;

    if(done or time >= 20.0)
    {
      return franka::MotionFinished(pose_desired);
    }

    return pose_desired;
  };


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

  robot->control(impedance_control_callback, cartesian_pose_callback);
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

