#include "iam_robolib/save_robot_state_data_to_shared_memory_buffer.h"

void save_robot_state_data_to_shared_memory_buffer(RunLoopSharedMemoryHandler* shared_memory_handler,
                                                   RobotStateData* robot_state_data,
                                                   int buffer_num) {
  if(buffer_num == 0) {
    if (robot_state_data->log_O_T_EE_0_.size() > 0 && robot_state_data->buffer_0_mutex_.try_lock()) {
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

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_O_T_EE_0_.back().data(), 
                                        robot_state_data->log_O_T_EE_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_O_T_EE_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_O_T_EE_d_0_.back().data(), 
                                        robot_state_data->log_O_T_EE_d_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_O_T_EE_d_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_F_T_EE_0_.back().data(), 
                                        robot_state_data->log_F_T_EE_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_F_T_EE_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_EE_T_K_0_.back().data(), 
                                        robot_state_data->log_EE_T_K_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_EE_T_K_0_.back().size();

        current_robot_state_buffer[buffer_idx++] = robot_state_data->log_m_ee_0_.back();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_I_ee_0_.back().data(), 
                                        robot_state_data->log_I_ee_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_I_ee_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_F_x_Cee_0_.back().data(), 
                                        robot_state_data->log_F_x_Cee_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_F_x_Cee_0_.back().size();

        current_robot_state_buffer[buffer_idx++] = robot_state_data->log_m_load_0_.back();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_I_load_0_.back().data(), 
                                        robot_state_data->log_I_load_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_I_load_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_F_x_Cload_0_.back().data(), 
                                        robot_state_data->log_F_x_Cload_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_F_x_Cload_0_.back().size();

        current_robot_state_buffer[buffer_idx++] = robot_state_data->log_m_total_0_.back();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_I_total_0_.back().data(), 
                                        robot_state_data->log_I_total_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_I_total_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_F_x_Ctotal_0_.back().data(), 
                                        robot_state_data->log_F_x_Ctotal_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_F_x_Ctotal_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_elbow_0_.back().data(), 
                                        robot_state_data->log_elbow_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_elbow_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_elbow_d_0_.back().data(), 
                                        robot_state_data->log_elbow_d_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_elbow_d_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_elbow_c_0_.back().data(), 
                                        robot_state_data->log_elbow_c_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_elbow_c_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_delbow_c_0_.back().data(), 
                                        robot_state_data->log_delbow_c_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_delbow_c_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_ddelbow_c_0_.back().data(), 
                                        robot_state_data->log_ddelbow_c_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_ddelbow_c_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_tau_J_0_.back().data(), 
                                        robot_state_data->log_tau_J_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_tau_J_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_tau_J_d_0_.back().data(), 
                                        robot_state_data->log_tau_J_d_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_tau_J_d_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_dtau_J_0_.back().data(), 
                                        robot_state_data->log_dtau_J_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_dtau_J_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_q_0_.back().data(), 
                                        robot_state_data->log_q_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_q_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_q_d_0_.back().data(), 
                                        robot_state_data->log_q_d_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_q_d_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_dq_0_.back().data(), 
                                        robot_state_data->log_dq_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_dq_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_dq_d_0_.back().data(), 
                                        robot_state_data->log_dq_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_dq_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_ddq_d_0_.back().data(), 
                                        robot_state_data->log_ddq_d_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_ddq_d_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_joint_contact_0_.back().data(), 
                                        robot_state_data->log_joint_contact_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_joint_contact_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_cartesian_contact_0_.back().data(), 
                                        robot_state_data->log_cartesian_contact_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_cartesian_contact_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_joint_collision_0_.back().data(), 
                                        robot_state_data->log_joint_collision_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_joint_collision_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_cartesian_collision_0_.back().data(), 
                                        robot_state_data->log_cartesian_collision_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_cartesian_collision_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_tau_ext_hat_filtered_0_.back().data(), 
                                        robot_state_data->log_tau_ext_hat_filtered_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_tau_ext_hat_filtered_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_O_F_ext_hat_K_0_.back().data(), 
                                        robot_state_data->log_O_F_ext_hat_K_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_O_F_ext_hat_K_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_K_F_ext_hat_K_0_.back().data(), 
                                        robot_state_data->log_K_F_ext_hat_K_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_K_F_ext_hat_K_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_O_dP_EE_d_0_.back().data(), 
                                        robot_state_data->log_O_dP_EE_d_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_O_dP_EE_d_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_O_T_EE_c_0_.back().data(), 
                                        robot_state_data->log_O_T_EE_c_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_O_T_EE_c_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_O_dP_EE_c_0_.back().data(), 
                                        robot_state_data->log_O_dP_EE_c_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_O_dP_EE_c_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_O_ddP_EE_c_0_.back().data(), 
                                        robot_state_data->log_O_ddP_EE_c_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_O_ddP_EE_c_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_theta_0_.back().data(), 
                                        robot_state_data->log_theta_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_theta_0_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_dtheta_0_.back().data(), 
                                        robot_state_data->log_dtheta_0_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_dtheta_0_.back().size();

        SharedBufferType shared_buffer_data_type_val = 0.0;
        std::array<bool, 37> bool_array_37;
        bool_array_37 = robot_state_data->log_current_errors_0_.back();
        for (size_t i = 0; i < bool_array_37.size(); i++) {
          shared_buffer_data_type_val = bool_array_37[i] ? 1 : 0;
          current_robot_state_buffer[buffer_idx++] = static_cast<SharedBufferType>(shared_buffer_data_type_val);
        }

        bool_array_37 = robot_state_data->log_last_motion_errors_0_.back();
        for (size_t i = 0; i < bool_array_37.size(); i++) {
          shared_buffer_data_type_val = bool_array_37[i] ? 1 : 0;
          current_robot_state_buffer[buffer_idx++] = static_cast<SharedBufferType>(shared_buffer_data_type_val);
        }

        current_robot_state_buffer[buffer_idx++] = robot_state_data->log_control_command_success_rate_0_.back();
        current_robot_state_buffer[buffer_idx++] = static_cast<SharedBufferType>(robot_state_data->log_robot_mode_0_.back());
        current_robot_state_buffer[buffer_idx++] = robot_state_data->log_robot_time_0_.back();

        current_robot_state_buffer[buffer_idx++] = robot_state_data->log_gripper_width_0_.back();
        current_robot_state_buffer[buffer_idx++] = robot_state_data->log_gripper_max_width_0_.back();
        current_robot_state_buffer[buffer_idx++] = robot_state_data->log_gripper_is_grasped_0_.back() ? 1.0 : 0.0;
        current_robot_state_buffer[buffer_idx++] = static_cast<SharedBufferType>(robot_state_data->log_gripper_temperature_0_.back());
        current_robot_state_buffer[buffer_idx++] = robot_state_data->log_gripper_time_0_.back();

        shared_memory_handler->getCurrentRobotStateBufferMutex()->unlock();
      }
      robot_state_data->buffer_0_mutex_.unlock();
    } else if (robot_state_data->log_O_T_EE_0_.size() > 0) {
      std::cout << "Get robot state failed to get lock 0\n";
    }
  }
  else {
    if (robot_state_data->log_O_T_EE_1_.size() > 0 && robot_state_data->buffer_1_mutex_.try_lock()) {
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

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_O_T_EE_1_.back().data(), 
                                        robot_state_data->log_O_T_EE_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_O_T_EE_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_O_T_EE_d_1_.back().data(), 
                                        robot_state_data->log_O_T_EE_d_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_O_T_EE_d_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_F_T_EE_1_.back().data(), 
                                        robot_state_data->log_F_T_EE_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_F_T_EE_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_EE_T_K_1_.back().data(), 
                                        robot_state_data->log_EE_T_K_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_EE_T_K_1_.back().size();

        current_robot_state_buffer[buffer_idx++] = robot_state_data->log_m_ee_1_.back();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_I_ee_1_.back().data(), 
                                        robot_state_data->log_I_ee_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_I_ee_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_F_x_Cee_1_.back().data(), 
                                        robot_state_data->log_F_x_Cee_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_F_x_Cee_1_.back().size();

        current_robot_state_buffer[buffer_idx++] = robot_state_data->log_m_load_1_.back();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_I_load_1_.back().data(), 
                                        robot_state_data->log_I_load_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_I_load_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_F_x_Cload_1_.back().data(), 
                                        robot_state_data->log_F_x_Cload_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_F_x_Cload_1_.back().size();

        current_robot_state_buffer[buffer_idx++] = robot_state_data->log_m_total_1_.back();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_I_total_1_.back().data(), 
                                        robot_state_data->log_I_total_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_I_total_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_F_x_Ctotal_1_.back().data(), 
                                        robot_state_data->log_F_x_Ctotal_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_F_x_Ctotal_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_elbow_1_.back().data(), 
                                        robot_state_data->log_elbow_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_elbow_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_elbow_d_1_.back().data(), 
                                        robot_state_data->log_elbow_d_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_elbow_d_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_elbow_c_1_.back().data(), 
                                        robot_state_data->log_elbow_c_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_elbow_c_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_delbow_c_1_.back().data(), 
                                        robot_state_data->log_delbow_c_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_delbow_c_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_ddelbow_c_1_.back().data(), 
                                        robot_state_data->log_ddelbow_c_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_ddelbow_c_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_tau_J_1_.back().data(), 
                                        robot_state_data->log_tau_J_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_tau_J_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_tau_J_d_1_.back().data(), 
                                        robot_state_data->log_tau_J_d_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_tau_J_d_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_dtau_J_1_.back().data(), 
                                        robot_state_data->log_dtau_J_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_dtau_J_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_q_1_.back().data(), 
                                        robot_state_data->log_q_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_q_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_q_d_1_.back().data(), 
                                        robot_state_data->log_q_d_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_q_d_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_dq_1_.back().data(), 
                                        robot_state_data->log_dq_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_dq_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_dq_d_1_.back().data(), 
                                        robot_state_data->log_dq_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_dq_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_ddq_d_1_.back().data(), 
                                        robot_state_data->log_ddq_d_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_ddq_d_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_joint_contact_1_.back().data(), 
                                        robot_state_data->log_joint_contact_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_joint_contact_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_cartesian_contact_1_.back().data(), 
                                        robot_state_data->log_cartesian_contact_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_cartesian_contact_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_joint_collision_1_.back().data(), 
                                        robot_state_data->log_joint_collision_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_joint_collision_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_cartesian_collision_1_.back().data(), 
                                        robot_state_data->log_cartesian_collision_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_cartesian_collision_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_tau_ext_hat_filtered_1_.back().data(), 
                                        robot_state_data->log_tau_ext_hat_filtered_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_tau_ext_hat_filtered_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_O_F_ext_hat_K_1_.back().data(), 
                                        robot_state_data->log_O_F_ext_hat_K_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_O_F_ext_hat_K_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_K_F_ext_hat_K_1_.back().data(), 
                                        robot_state_data->log_K_F_ext_hat_K_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_K_F_ext_hat_K_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_O_dP_EE_d_1_.back().data(), 
                                        robot_state_data->log_O_dP_EE_d_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_O_dP_EE_d_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_O_T_EE_c_1_.back().data(), 
                                        robot_state_data->log_O_T_EE_c_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_O_T_EE_c_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_O_dP_EE_c_1_.back().data(), 
                                        robot_state_data->log_O_dP_EE_c_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_O_dP_EE_c_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_O_ddP_EE_c_1_.back().data(), 
                                        robot_state_data->log_O_ddP_EE_c_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_O_ddP_EE_c_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_theta_1_.back().data(), 
                                        robot_state_data->log_theta_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_theta_1_.back().size();

        memcpy(&current_robot_state_buffer[buffer_idx], robot_state_data->log_dtheta_1_.back().data(), 
                                        robot_state_data->log_dtheta_1_.back().size() * sizeof(SharedBufferType));
        buffer_idx += robot_state_data->log_dtheta_1_.back().size();

        SharedBufferType shared_buffer_data_type_val = 0.0;
        std::array<bool, 37> bool_array_37;
        bool_array_37 = robot_state_data->log_current_errors_1_.back();
        for (size_t i = 0; i < bool_array_37.size(); i++) {
          shared_buffer_data_type_val = bool_array_37[i] ? 1.0 : 0.0;
          current_robot_state_buffer[buffer_idx++] = static_cast<SharedBufferType>(shared_buffer_data_type_val);
        }

        bool_array_37 = robot_state_data->log_last_motion_errors_1_.back();
        for (size_t i = 0; i < bool_array_37.size(); i++) {
          shared_buffer_data_type_val = bool_array_37[i] ? 1.0 : 0.0;
          current_robot_state_buffer[buffer_idx++] = static_cast<SharedBufferType>(shared_buffer_data_type_val);
        }

        current_robot_state_buffer[buffer_idx++] = robot_state_data->log_control_command_success_rate_1_.back();
        current_robot_state_buffer[buffer_idx++] = static_cast<SharedBufferType>(robot_state_data->log_robot_mode_1_.back());
        current_robot_state_buffer[buffer_idx++] = robot_state_data->log_robot_time_1_.back();

        current_robot_state_buffer[buffer_idx++] = robot_state_data->log_gripper_width_1_.back();
        current_robot_state_buffer[buffer_idx++] = robot_state_data->log_gripper_max_width_1_.back();
        current_robot_state_buffer[buffer_idx++] = robot_state_data->log_gripper_is_grasped_1_.back() ? 1.0 : 0.0;
        current_robot_state_buffer[buffer_idx++] = static_cast<SharedBufferType>(robot_state_data->log_gripper_temperature_1_.back());
        current_robot_state_buffer[buffer_idx++] = robot_state_data->log_gripper_time_1_.back();

        shared_memory_handler->getCurrentRobotStateBufferMutex()->unlock();
      }
      robot_state_data->buffer_1_mutex_.unlock();
    } else if (robot_state_data->log_O_T_EE_1_.size() > 0) {
      std::cout << "Get robot state failed to get lock 1\n";
    }
  }  
}