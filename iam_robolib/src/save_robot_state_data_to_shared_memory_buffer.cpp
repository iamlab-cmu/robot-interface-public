#include "iam_robolib/save_robot_state_data_to_shared_memory_buffer.h"

void save_current_robot_state_data_to_shared_memory_buffer(RunLoopSharedMemoryHandler* shared_memory_handler,
                                                           RobotStateData* robot_state_data) {
  if (shared_memory_handler->getCurrentRobotStateBufferMutex()->try_lock()) {
    SharedBufferTypePtr current_robot_state_buffer = shared_memory_handler->getCurrentRobotStateBuffer();
    size_t buffer_idx = 0;

    current_robot_state_buffer[buffer_idx++] = static_cast<SharedBufferType>(16+16+16+16+
                                                                             1+9+3+
                                                                             1+9+3+
                                                                             1+9+3+
                                                                             2+2+2+2+2+
                                                                             7+7+7+7+7+7+7+7+
                                                                             7+6+7+6+
                                                                             7+6+6+6+16+6+6+
                                                                             7+7+37+37+1+1+1+
                                                                             1+1+1+1+1); // 344

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.O_T_EE.data(), 
                                    robot_state_data->current_robot_state.O_T_EE.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.O_T_EE.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.O_T_EE_d.data(), 
                                    robot_state_data->current_robot_state.O_T_EE_d.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.O_T_EE_d.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.F_T_EE.data(), 
                                    robot_state_data->current_robot_state.F_T_EE.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.F_T_EE.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.EE_T_K.data(), 
                                    robot_state_data->current_robot_state.EE_T_K.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.EE_T_K.size();

    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.m_ee;

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.I_ee.data(), 
                                    robot_state_data->current_robot_state.I_ee.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.I_ee.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.F_x_Cee.data(), 
                                    robot_state_data->current_robot_state.F_x_Cee.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.F_x_Cee.size();

    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.m_load;

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.I_load.data(), 
                                    robot_state_data->current_robot_state.I_load.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.I_load.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.F_x_Cload.data(), 
                                    robot_state_data->current_robot_state.F_x_Cload.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.F_x_Cload.size();

    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.m_total;

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.I_total.data(), 
                                    robot_state_data->current_robot_state.I_total.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.I_total.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.F_x_Ctotal.data(), 
                                    robot_state_data->current_robot_state.F_x_Ctotal.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.F_x_Ctotal.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.elbow.data(), 
                                    robot_state_data->current_robot_state.elbow.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.elbow.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.elbow_d.data(), 
                                    robot_state_data->current_robot_state.elbow_d.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.elbow_d.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.elbow_c.data(), 
                                    robot_state_data->current_robot_state.elbow_c.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.elbow_c.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.delbow_c.data(), 
                                    robot_state_data->current_robot_state.delbow_c.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.delbow_c.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.ddelbow_c.data(), 
                                    robot_state_data->current_robot_state.ddelbow_c.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.ddelbow_c.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.tau_J.data(), 
                                    robot_state_data->current_robot_state.tau_J.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.tau_J.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.tau_J_d.data(), 
                                    robot_state_data->current_robot_state.tau_J_d.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.tau_J_d.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.dtau_J.data(), 
                                    robot_state_data->current_robot_state.dtau_J.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.dtau_J.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.q.data(), 
                                    robot_state_data->current_robot_state.q.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.q.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.q_d.data(), 
                                    robot_state_data->current_robot_state.q_d.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.q_d.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.dq.data(), 
                                    robot_state_data->current_robot_state.dq.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.dq.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.dq_d.data(), 
                                    robot_state_data->current_robot_state.dq.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.dq.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.ddq_d.data(), 
                                    robot_state_data->current_robot_state.ddq_d.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.ddq_d.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.joint_contact.data(), 
                                    robot_state_data->current_robot_state.joint_contact.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.joint_contact.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.cartesian_contact.data(), 
                                    robot_state_data->current_robot_state.cartesian_contact.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.cartesian_contact.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.joint_collision.data(), 
                                    robot_state_data->current_robot_state.joint_collision.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.joint_collision.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.cartesian_collision.data(), 
                                    robot_state_data->current_robot_state.cartesian_collision.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.cartesian_collision.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.tau_ext_hat_filtered.data(), 
                                    robot_state_data->current_robot_state.tau_ext_hat_filtered.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.tau_ext_hat_filtered.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.O_F_ext_hat_K.data(), 
                                    robot_state_data->current_robot_state.O_F_ext_hat_K.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.O_F_ext_hat_K.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.K_F_ext_hat_K.data(), 
                                    robot_state_data->current_robot_state.K_F_ext_hat_K.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.K_F_ext_hat_K.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.O_dP_EE_d.data(), 
                                    robot_state_data->current_robot_state.O_dP_EE_d.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.O_dP_EE_d.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.O_T_EE_c.data(), 
                                    robot_state_data->current_robot_state.O_T_EE_c.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.O_T_EE_c.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.O_dP_EE_c.data(), 
                                    robot_state_data->current_robot_state.O_dP_EE_c.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.O_dP_EE_c.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.O_ddP_EE_c.data(), 
                                    robot_state_data->current_robot_state.O_ddP_EE_c.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.O_ddP_EE_c.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.theta.data(), 
                                    robot_state_data->current_robot_state.theta.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.theta.size();

    memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->current_robot_state.dtheta.data(), 
                                    robot_state_data->current_robot_state.dtheta.size() * sizeof(SharedBufferType));
    buffer_idx += robot_state_data->current_robot_state.dtheta.size();

    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.joint_position_limits_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.cartesian_position_limits_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.self_collision_avoidance_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.joint_velocity_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.cartesian_velocity_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.force_control_safety_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.joint_reflex? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.cartesian_reflex? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.max_goal_pose_deviation_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.max_path_pose_deviation_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.cartesian_velocity_profile_safety_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.joint_position_motion_generator_start_pose_invalid? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.joint_motion_generator_position_limits_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.joint_motion_generator_velocity_limits_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.joint_motion_generator_velocity_discontinuity? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.joint_motion_generator_acceleration_discontinuity? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.cartesian_position_motion_generator_start_pose_invalid? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.cartesian_motion_generator_elbow_limit_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.cartesian_motion_generator_velocity_limits_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.cartesian_motion_generator_velocity_discontinuity? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.cartesian_motion_generator_acceleration_discontinuity? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.cartesian_motion_generator_elbow_sign_inconsistent? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.cartesian_motion_generator_start_elbow_invalid? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.cartesian_motion_generator_joint_position_limits_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.cartesian_motion_generator_joint_velocity_limits_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.cartesian_motion_generator_joint_velocity_discontinuity? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.cartesian_motion_generator_joint_acceleration_discontinuity? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.cartesian_position_motion_generator_invalid_frame? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.force_controller_desired_force_tolerance_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.controller_torque_discontinuity? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.start_elbow_sign_inconsistent? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.communication_constraints_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.power_limit_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.joint_p2p_insufficient_torque_for_planning? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.tau_j_range_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.instability_detected? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.current_errors.joint_move_in_wrong_direction? 1.0 : 0.0;

    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.joint_position_limits_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.cartesian_position_limits_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.self_collision_avoidance_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.joint_velocity_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.cartesian_velocity_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.force_control_safety_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.joint_reflex? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.cartesian_reflex? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.max_goal_pose_deviation_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.max_path_pose_deviation_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.cartesian_velocity_profile_safety_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.joint_position_motion_generator_start_pose_invalid? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.joint_motion_generator_position_limits_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.joint_motion_generator_velocity_limits_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.joint_motion_generator_velocity_discontinuity? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.joint_motion_generator_acceleration_discontinuity? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.cartesian_position_motion_generator_start_pose_invalid? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.cartesian_motion_generator_elbow_limit_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.cartesian_motion_generator_velocity_limits_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.cartesian_motion_generator_velocity_discontinuity? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.cartesian_motion_generator_acceleration_discontinuity? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.cartesian_motion_generator_elbow_sign_inconsistent? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.cartesian_motion_generator_start_elbow_invalid? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.cartesian_motion_generator_joint_position_limits_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.cartesian_motion_generator_joint_velocity_limits_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.cartesian_motion_generator_joint_velocity_discontinuity? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.cartesian_motion_generator_joint_acceleration_discontinuity? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.cartesian_position_motion_generator_invalid_frame? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.force_controller_desired_force_tolerance_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.controller_torque_discontinuity? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.start_elbow_sign_inconsistent? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.communication_constraints_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.power_limit_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.joint_p2p_insufficient_torque_for_planning? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.tau_j_range_violation? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.instability_detected? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.last_motion_errors.joint_move_in_wrong_direction? 1.0 : 0.0;

    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.control_command_success_rate;
    current_robot_state_buffer[buffer_idx++] = static_cast<SharedBufferType>(robot_state_data->current_robot_state.robot_mode);
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_robot_state.time.toSec();

    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_gripper_state.width;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_gripper_state.max_width;
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_gripper_state.is_grasped ? 1.0 : 0.0;
    current_robot_state_buffer[buffer_idx++] = static_cast<SharedBufferType>(robot_state_data->current_gripper_state.temperature);
    current_robot_state_buffer[buffer_idx++] = robot_state_data->current_gripper_state.time.toSec();

    shared_memory_handler->getCurrentRobotStateBufferMutex()->unlock();
  }
}