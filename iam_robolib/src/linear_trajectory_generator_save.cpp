//
// Created by Kevin on 11/29/18.
//

#include "TrajectoryGenerator/linear_trajectory_generator.h"

void LinearTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  model_ = robot.loadModel();

  num_params = static_cast<int>params_[1];

  if(num_params != 16)
  {
    std::cout << "Incorrect number of params given: " << num_params << std::endl;
  }

  memcpy(deltas_, &params_[2], num_deltas_ * sizeof(float));

  memcpy(k_gains_, &params_[18], num_gains_ * sizeof(float));

  memcpy(d_gains_, &params_[25], num_gains_ * sizeof(float));

  cartesian_pose_callback_ = [=, &time_, &running_, &num_deltas_, &deltas_, &initial_pose_, &pose_desired](
                                   const franka::RobotState& robot_state,
                                   franka::Duration period) -> franka::CartesianPose {
    // Update time.
    time += period.toSec();

    if (time == 0.0) {
      // Read the initial pose to start the motion from the first time step.
      initial_pose_ = robot_state.O_T_EE_c;
      pose_desired = initial_pose_;
    }

    for(int i = 0; i < num_deltas_; i++)
    {
      pose_desired.O_T_EE[i] += deltas_[i];
    }

    // Send desired pose.
    if (running_ == false) {
      return franka::MotionFinished(pose_desired);
    }

    return pose_desired;
  };

  impedance_control_callback_ = [&model_, &k_gains_, &d_gains_](
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
}

void LinearTrajectoryGenerator::initialize_trajectory() {
  
}

void LinearTrajectoryGenerator::get_next_step() {

  
}



