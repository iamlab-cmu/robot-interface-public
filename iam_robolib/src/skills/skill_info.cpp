//
// Created by mohit on 11/20/18.
//

#include "iam_robolib/skills/skill_info.h"

#include <cassert>
#include <iostream>
#include <vector>
#include <array>

#include <Eigen/Dense>

#include <franka/exception.h>

#include "iam_robolib/run_loop.h"
#include "iam_robolib/robot_state_data.h"
#include "iam_robolib/run_loop_shared_memory_handler.h"

#include <iam_robolib_common/run_loop_process_info.h>

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

void SkillInfo::execute_skill_on_franka(run_loop* run_loop, 
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

  // define callback for the torque control loop
  std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
      impedance_control_callback = [&](const franka::RobotState& robot_state,
                                              franka::Duration period) -> franka::Torques {

    if (time == 0.0) {
      traj_generator_->initialize_trajectory(robot_state);
    }

    if (robot_state_data->mutex_.try_lock()) {
      robot_state_data->counter_ += 1;
      robot_state_data->time_ =  period.toSec();
      robot_state_data->has_data_ = true;
      robot_state_data->mutex_.unlock();
    }

    traj_generator_->dt_ = period.toSec();
    traj_generator_->time_ += period.toSec();
    time += period.toSec();
    log_counter += 1;

    traj_generator_->get_next_step();

    if (log_counter % 1 == 0) {
      robot_state_data->log_pose_desired(traj_generator_->pose_desired_);
      robot_state_data->log_robot_state(robot_state, time);
    }

    feedback_controller_->get_next_step(robot_state, traj_generator_);

    bool done = termination_handler_->should_terminate_on_franka(robot_state, traj_generator_);

    try {
      if (lock.try_lock()) {
        run_loop_info->set_time_since_skill_started(time);
        run_loop_info->set_robot_time(robot_state.time.toSec());
      } 
    } catch (boost::interprocess::lock_exception) {
      // Do nothing
    }

    if(done) {
      return franka::MotionFinished(franka::Torques(feedback_controller_->tau_d_array_));
    }

    return feedback_controller_->tau_d_array_;
  };

  robot->robot_.control(impedance_control_callback);
}

void SkillInfo::execute_skill_on_franka_joint_base(run_loop* run_loop, 
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

  franka::Model *model = robot->getModel();

  // define callback for the torque control loop
  std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
      impedance_control_callback = [&, &time](const franka::RobotState& robot_state,
                                              franka::Duration period) -> franka::Torques {

    if (time == 0.0) {
      traj_generator_->initialize_trajectory(robot_state);
      feedback_controller_->initialize_controller(model);
    }

    if (robot_state_data->mutex_.try_lock()) {
      robot_state_data->counter_ += 1;
      robot_state_data->time_ =  period.toSec();
      robot_state_data->has_data_ = true;
      robot_state_data->mutex_.unlock();
    }

    traj_generator_->dt_ = period.toSec();
    traj_generator_->time_ += period.toSec();
    time += period.toSec();
    log_counter += 1;

    traj_generator_->get_next_step();

    if (log_counter % 1 == 0) {
      robot_state_data->log_pose_desired(traj_generator_->pose_desired_);
      robot_state_data->log_robot_state(robot_state, time);
    }

    feedback_controller_->get_next_step(robot_state, traj_generator_);

    bool done = termination_handler_->should_terminate_on_franka(robot_state, traj_generator_);

    if (done) {
      // return 0 torques to finish
      std::array<double, 7> tau_d_array{};
      franka::Torques torques(tau_d_array);
      return franka::MotionFinished(torques);
    }

    if(done) {
      return franka::MotionFinished(franka::Torques(feedback_controller_->tau_d_array_));
    }


    return feedback_controller_->tau_d_array_;
  };

  robot->robot_.control(impedance_control_callback);
}

void SkillInfo::execute_skill_on_franka_temp2(run_loop* run_loop, 
                                              FrankaRobot* robot,
                                              RobotStateData *robot_state_data) {
  const double translational_stiffness{500.0};
  const double rotational_stiffness{35.0};
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
      Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
      Eigen::MatrixXd::Identity(3, 3);

  double time = 0.0;
  int log_counter = 0;

  std::cout << "Will run the control loop\n";

  franka::Model *model = robot->getModel();

  // define callback for the torque control loop
  std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
      impedance_control_callback = [&, &time](const franka::RobotState& robot_state,
                                              franka::Duration period) -> franka::Torques {

    if (time == 0.0) {
      traj_generator_->initialize_trajectory(robot_state);
    }

    if (robot_state_data->mutex_.try_lock()) {
      robot_state_data->counter_ += 1;
      robot_state_data->time_ =  period.toSec();
      robot_state_data->has_data_ = true;
      robot_state_data->mutex_.unlock();
    }

    traj_generator_->dt_ = period.toSec();
    traj_generator_->time_ += period.toSec();
    time += period.toSec();
    log_counter += 1;

    traj_generator_->get_next_step();

    bool done = termination_handler_->should_terminate(traj_generator_);

    if (log_counter % 1 == 0) {
      robot_state_data->log_pose_desired(traj_generator_->pose_desired_);
      robot_state_data->log_robot_state(robot_state, time);
    }

    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(traj_generator_->pose_desired_.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.linear());

    // get state variables
    std::array<double, 7> coriolis_array = model->coriolis(robot_state);
    std::array<double, 42> jacobian_array =
        model->zeroJacobian(franka::Frame::kEndEffector, robot_state);

    // convert to Eigen
    Eigen::Map<const Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());

    // compute error to desired equilibrium pose
    // position error
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d;

    // orientation error
    // "difference" quaternion
    if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() << -orientation.coeffs();
    }
    Eigen::Quaterniond error_quaternion(orientation * orientation_d.inverse());
    // convert to axis angle
    Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
    // compute "orientation error"
    error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

    // compute control
    Eigen::VectorXd tau_task(7), tau_d(7);

    // Spring damper system with damping ratio=1
    tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
    tau_d << tau_task + coriolis;

    std::array<double, 7> tau_d_array{};
    Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
    return tau_d_array;
  };

  robot->robot_.control(impedance_control_callback);
}

void SkillInfo::execute_skill_on_franka_temp(run_loop* run_loop, 
                                             FrankaRobot* robot,
                                             RobotStateData *robot_state_data) {
  double time = 0.0;
  int log_counter = 0;

  std::cout << "Will run the control loop\n";
  std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)> cartesian_pose_callback =
      [=, &time, &log_counter](const franka::RobotState& robot_state,
                                franka::Duration period) -> franka::CartesianPose {
        if (time == 0.0) {
          traj_generator_->initialize_trajectory(robot_state);
        }

        if (robot_state_data->mutex_.try_lock()) {
          robot_state_data->counter_ += 1;
          robot_state_data->time_ =  period.toSec();
          robot_state_data->has_data_ = true;
          robot_state_data->mutex_.unlock();
        }

        traj_generator_->dt_ = period.toSec();
        traj_generator_->time_ += period.toSec();
        time += period.toSec();
        log_counter += 1;

        traj_generator_->get_next_step();

        bool done = termination_handler_->should_terminate(traj_generator_);

        franka::CartesianPose pose_desired(traj_generator_->pose_desired_);
        if (log_counter % 1 == 0) {
          robot_state_data->log_pose_desired(traj_generator_->pose_desired_);
          robot_state_data->log_robot_state(robot_state, time);
        }

        if(done or time >= traj_generator_->run_time_ + traj_generator_->acceleration_time_)
        {
          return franka::MotionFinished(pose_desired);
        }

        return pose_desired;
      };

  std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)>
      joint_pose_callback = [=, &time, &log_counter](
          const franka::RobotState& robot_state,
          franka::Duration period) -> franka::JointPositions {
    if (time == 0.0) {
      traj_generator_->initialize_trajectory(robot_state);
    }
    time += period.toSec();
    traj_generator_->time_ = time;
    traj_generator_->dt_ = period.toSec();
    traj_generator_->get_next_step();

    bool done = termination_handler_->should_terminate(traj_generator_);
    franka::JointPositions joint_desired(traj_generator_->joint_desired_);

    log_counter += 1;
    if (log_counter % 1 == 0) {
      robot_state_data->log_pose_desired(traj_generator_->pose_desired_);
      robot_state_data->log_robot_state(robot_state, time);
    }

    if(done or time >= traj_generator_->run_time_) {
      return franka::MotionFinished(joint_desired);
    }
    return joint_desired;
  };

  franka::Model *model = robot->getModel();

  std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  std::function<franka::Torques(const franka::RobotState&, franka::Duration)> impedance_control_callback =
      [=, model, &q_goal](
          const franka::RobotState& state, franka::Duration /*period*/) -> franka::Torques {
        // Read current coriolis terms from model.
        std::array<double, 7> coriolis = model->coriolis(state);

        // Compute torque command from joint impedance control law.
        // Note: The answer to our Cartesian pose inverse kinematics is always in state.q_d with one
        // time step delay.
        std::array<double, 7> tau_d_calculated;
        for (size_t i = 0; i < 7; i++) {
          tau_d_calculated[i] =
              k_gains_[i] * (state.q_d[i] - state.q[i]) - d_gains_[i] * state.dq[i] + coriolis[i];
          // tau_d_calculated[i] =
          //     k_gains_[i] * (q_goal[i] - state.q[i]) - d_gains_[i] * state.dq[i] + coriolis[i];
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

  robot->robot_.control()*/

  // robot->robot_.control(impedance_control_callback, cartesian_pose_callback);
  // robot->robot_.control(cartesian_pose_callback, franka::ControllerMode::kCartesianImpedance, true, 1000.0);
  // robot->robot_.control(impedance_control_callback, joint_pose_callback);
  robot->robot_.control(impedance_control_callback);
}

void SkillInfo::write_result_to_shared_memory(double *result_buffer) {
  std::cout << "Should write result to shared memory\n";
}

void SkillInfo::write_result_to_shared_memory(double *result_buffer, FrankaRobot* robot) {
  franka::GripperState gripper_state = robot->getGripperState();
  franka::RobotState robot_state = robot->getRobotState();

  int result_buffer_idx = 0;

  result_buffer[result_buffer_idx++] = static_cast<double>(16+16+16+16+
                                                           1+9+3+
                                                           1+9+3+
                                                           1+9+3+
                                                           2+2+2+2+2+
                                                           7+7+7+7+7+7+7+7+
                                                           7+6+7+6+
                                                           7+6+6+6+16+6+6+
                                                           7+7+37+37+
                                                           1+1+1+
                                                           1+1+1+1+1); // 344

  memcpy(&result_buffer[result_buffer_idx], robot_state.O_T_EE.data(), robot_state.O_T_EE.size() * sizeof(double));
  result_buffer_idx += robot_state.O_T_EE.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.O_T_EE_d.data(), robot_state.O_T_EE_d.size() * sizeof(double));
  result_buffer_idx += robot_state.O_T_EE_d.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.F_T_EE.data(), robot_state.F_T_EE.size() * sizeof(double));
  result_buffer_idx += robot_state.F_T_EE.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.EE_T_K.data(), robot_state.EE_T_K.size() * sizeof(double));
  result_buffer_idx += robot_state.EE_T_K.size();

  result_buffer[result_buffer_idx++] = robot_state.m_ee;

  memcpy(&result_buffer[result_buffer_idx], robot_state.I_ee.data(), robot_state.I_ee.size() * sizeof(double));
  result_buffer_idx += robot_state.I_ee.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.F_x_Cee.data(), robot_state.F_x_Cee.size() * sizeof(double));
  result_buffer_idx += robot_state.F_x_Cee.size();

  result_buffer[result_buffer_idx++] = robot_state.m_load;

  memcpy(&result_buffer[result_buffer_idx], robot_state.I_load.data(), robot_state.I_load.size() * sizeof(double));
  result_buffer_idx += robot_state.I_load.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.F_x_Cload.data(), robot_state.F_x_Cload.size() * sizeof(double));
  result_buffer_idx += robot_state.F_x_Cload.size();

  result_buffer[result_buffer_idx++] = robot_state.m_total;

  memcpy(&result_buffer[result_buffer_idx], robot_state.I_total.data(), robot_state.I_total.size() * sizeof(double));
  result_buffer_idx += robot_state.I_total.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.F_x_Ctotal.data(), robot_state.F_x_Ctotal.size() * sizeof(double));
  result_buffer_idx += robot_state.F_x_Ctotal.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.elbow.data(), robot_state.elbow.size() * sizeof(double));
  result_buffer_idx += robot_state.elbow.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.elbow_d.data(), robot_state.elbow_d.size() * sizeof(double));
  result_buffer_idx += robot_state.elbow_d.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.elbow_c.data(), robot_state.elbow_c.size() * sizeof(double));
  result_buffer_idx += robot_state.elbow_c.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.delbow_c.data(), robot_state.delbow_c.size() * sizeof(double));
  result_buffer_idx += robot_state.delbow_c.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.ddelbow_c.data(), robot_state.ddelbow_c.size() * sizeof(double));
  result_buffer_idx += robot_state.ddelbow_c.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.tau_J.data(), robot_state.tau_J.size() * sizeof(double));
  result_buffer_idx += robot_state.tau_J.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.tau_J_d.data(), robot_state.tau_J_d.size() * sizeof(double));
  result_buffer_idx += robot_state.tau_J_d.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.dtau_J.data(), robot_state.dtau_J.size() * sizeof(double));
  result_buffer_idx += robot_state.dtau_J.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.q.data(), robot_state.q.size() * sizeof(double));
  result_buffer_idx += robot_state.q.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.q_d.data(), robot_state.q_d.size() * sizeof(double));
  result_buffer_idx += robot_state.q_d.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.dq.data(), robot_state.dq.size() * sizeof(double));
  result_buffer_idx += robot_state.dq.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.dq_d.data(), robot_state.dq_d.size() * sizeof(double));
  result_buffer_idx += robot_state.dq_d.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.ddq_d.data(), robot_state.ddq_d.size() * sizeof(double));
  result_buffer_idx += robot_state.ddq_d.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.joint_contact.data(), robot_state.joint_contact.size() * sizeof(double));
  result_buffer_idx += robot_state.joint_contact.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.cartesian_contact.data(), robot_state.cartesian_contact.size() * sizeof(double));
  result_buffer_idx += robot_state.cartesian_contact.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.joint_collision.data(), robot_state.joint_collision.size() * sizeof(double));
  result_buffer_idx += robot_state.joint_collision.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.cartesian_collision.data(), robot_state.cartesian_collision.size() * sizeof(double));
  result_buffer_idx += robot_state.cartesian_collision.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.tau_ext_hat_filtered.data(), robot_state.tau_ext_hat_filtered.size() * sizeof(double));
  result_buffer_idx += robot_state.tau_ext_hat_filtered.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.O_F_ext_hat_K.data(), robot_state.O_F_ext_hat_K.size() * sizeof(double));
  result_buffer_idx += robot_state.O_F_ext_hat_K.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.K_F_ext_hat_K.data(), robot_state.K_F_ext_hat_K.size() * sizeof(double));
  result_buffer_idx += robot_state.K_F_ext_hat_K.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.O_dP_EE_d.data(), robot_state.O_dP_EE_d.size() * sizeof(double));
  result_buffer_idx += robot_state.O_dP_EE_d.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.O_T_EE_c.data(), robot_state.O_T_EE_c.size() * sizeof(double));
  result_buffer_idx += robot_state.O_T_EE_c.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.O_dP_EE_c.data(), robot_state.O_dP_EE_c.size() * sizeof(double));
  result_buffer_idx += robot_state.O_dP_EE_c.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.O_ddP_EE_c.data(), robot_state.O_ddP_EE_c.size() * sizeof(double));
  result_buffer_idx += robot_state.O_ddP_EE_c.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.theta.data(), robot_state.theta.size() * sizeof(double));
  result_buffer_idx += robot_state.theta.size();

  memcpy(&result_buffer[result_buffer_idx], robot_state.dtheta.data(), robot_state.dtheta.size() * sizeof(double));
  result_buffer_idx += robot_state.dtheta.size();

  result_buffer[result_buffer_idx++] = robot_state.current_errors.joint_position_limits_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.cartesian_position_limits_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.self_collision_avoidance_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.joint_velocity_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.cartesian_velocity_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.force_control_safety_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.joint_reflex ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.cartesian_reflex ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.max_goal_pose_deviation_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.max_path_pose_deviation_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.cartesian_velocity_profile_safety_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.joint_position_motion_generator_start_pose_invalid ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.joint_motion_generator_position_limits_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.joint_motion_generator_velocity_limits_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.joint_motion_generator_velocity_discontinuity ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.joint_motion_generator_acceleration_discontinuity ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.cartesian_position_motion_generator_start_pose_invalid ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.cartesian_motion_generator_elbow_limit_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.cartesian_motion_generator_velocity_limits_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.cartesian_motion_generator_velocity_discontinuity ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.cartesian_motion_generator_acceleration_discontinuity ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.cartesian_motion_generator_elbow_sign_inconsistent ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.cartesian_motion_generator_start_elbow_invalid ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.cartesian_motion_generator_joint_position_limits_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.cartesian_motion_generator_joint_velocity_limits_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.cartesian_motion_generator_joint_velocity_discontinuity ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.cartesian_motion_generator_joint_acceleration_discontinuity ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.cartesian_position_motion_generator_invalid_frame ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.force_controller_desired_force_tolerance_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.controller_torque_discontinuity ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.start_elbow_sign_inconsistent ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.communication_constraints_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.power_limit_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.joint_p2p_insufficient_torque_for_planning ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.tau_j_range_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.instability_detected ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.current_errors.joint_move_in_wrong_direction ? 1.0 : 0.0;

  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.joint_position_limits_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.cartesian_position_limits_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.self_collision_avoidance_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.joint_velocity_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.cartesian_velocity_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.force_control_safety_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.joint_reflex ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.cartesian_reflex ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.max_goal_pose_deviation_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.max_path_pose_deviation_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.cartesian_velocity_profile_safety_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.joint_position_motion_generator_start_pose_invalid ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.joint_motion_generator_position_limits_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.joint_motion_generator_velocity_limits_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.joint_motion_generator_velocity_discontinuity ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.joint_motion_generator_acceleration_discontinuity ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.cartesian_position_motion_generator_start_pose_invalid ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.cartesian_motion_generator_elbow_limit_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.cartesian_motion_generator_velocity_limits_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.cartesian_motion_generator_velocity_discontinuity ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.cartesian_motion_generator_acceleration_discontinuity ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.cartesian_motion_generator_elbow_sign_inconsistent ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.cartesian_motion_generator_start_elbow_invalid ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.cartesian_motion_generator_joint_position_limits_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.cartesian_motion_generator_joint_velocity_limits_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.cartesian_motion_generator_joint_velocity_discontinuity ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.cartesian_motion_generator_joint_acceleration_discontinuity ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.cartesian_position_motion_generator_invalid_frame ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.force_controller_desired_force_tolerance_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.controller_torque_discontinuity ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.start_elbow_sign_inconsistent ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.communication_constraints_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.power_limit_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.joint_p2p_insufficient_torque_for_planning ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.tau_j_range_violation ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.instability_detected ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = robot_state.last_motion_errors.joint_move_in_wrong_direction ? 1.0 : 0.0;

  result_buffer[result_buffer_idx++] = robot_state.control_command_success_rate;
  result_buffer[result_buffer_idx++] = static_cast<double>(static_cast<uint8_t>(robot_state.robot_mode));
  result_buffer[result_buffer_idx++] = robot_state.time.toSec();

  result_buffer[result_buffer_idx++] = gripper_state.width;
  result_buffer[result_buffer_idx++] = gripper_state.max_width;
  result_buffer[result_buffer_idx++] = gripper_state.is_grasped ? 1.0 : 0.0;
  result_buffer[result_buffer_idx++] = static_cast<double>(gripper_state.temperature);
  result_buffer[result_buffer_idx++] = gripper_state.time.toSec();
}

void SkillInfo::write_result_to_shared_memory(double *result_buffer, Robot* robot) {
  std::cout << "Writing final robot state to shared memory\n";

  switch(robot->robot_type_)
  {
    case RobotType::FRANKA:
      write_result_to_shared_memory(result_buffer, dynamic_cast<FrankaRobot *>(robot));
      break;
    case RobotType::UR5E:
      break;
  }
  
}

void SkillInfo::write_feedback_to_shared_memory(double *feedback_buffer) {
  std::cout << "Should write feedback to shared memory\n";
}
