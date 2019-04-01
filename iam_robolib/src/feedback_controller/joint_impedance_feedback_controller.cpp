#include "iam_robolib/feedback_controller/joint_impedance_feedback_controller.h"

#include <iostream>

#include <franka/rate_limiting.h>

#include "iam_robolib/trajectory_generator/joint_trajectory_generator.h"

void JointImpedanceFeedbackController::parse_parameters() {
  // First parameter is reserved for the type

  int num_params = static_cast<int>(params_[1]);

  // No parameters given, using default translational stiffness and rotational stiffness
  if (num_params == 0) {
    std::cout << "No parameters given, using default translational and rotational stiffness."
              << std::endl;
  }
  // translational_stiffness(1) and rotational_stiffness(1) were given
  if (num_params == 14) {
    for(int i = 0; i < 7; i++) {
      k_gains_[i] = static_cast<double>(params_[2+i]);
    }

    for(int i = 0; i < 7; i++) {
      d_gains_[i] = static_cast<double>(params_[9+i]);
    }
  } else {
    std::cout << "Invalid number of params provided: " << num_params << std::endl;
  }
}

void JointImpedanceFeedbackController::initialize_controller() {}

void JointImpedanceFeedbackController::initialize_controller(FrankaRobot *robot) {
  model_ = robot->getModel();
}

void JointImpedanceFeedbackController::get_next_step() {}

void JointImpedanceFeedbackController::get_next_step(const franka::RobotState &robot_state,
                                                     TrajectoryGenerator *traj_generator) {

  // Read current coriolis terms from model.
  std::array<double, 7> coriolis = model_->coriolis(robot_state);

  JointTrajectoryGenerator* joint_trajectory_generator = dynamic_cast<JointTrajectoryGenerator*>(traj_generator);

  if(joint_trajectory_generator == nullptr) {
    throw std::bad_cast();
  }

  std::array<double, 7> desired_joints = joint_trajectory_generator->get_desired_joints();

  // Compute torque command from joint impedance control law.
  // Note: The answer to our Cartesian pose inverse kinematics is always in state.q_d with one
  // time step delay.
  std::array<double, 7> tau_d_calculated;
  for (size_t i = 0; i < 7; i++) {
    tau_d_calculated[i] = k_gains_[i] * (desired_joints[i] - robot_state.q[i])
          - d_gains_[i] * robot_state.dq[i] + coriolis[i];
  }

  // The following line is only necessary if rate limiting is not activate. If we activated
  // rate limiting for the control loop (activated by default), the torque would anyway be
  // adjusted!
  std::array<double, 7> tau_d_rate_limited =
      franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, robot_state.tau_J_d);

  tau_d_array_ = tau_d_rate_limited;
}
