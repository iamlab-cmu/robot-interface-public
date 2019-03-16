//
// Created by mohit on 12/5/18.
//

#include "iam_robolib/file_stream_logger.h"

#include <iostream>

bool FileStreamLogger::writeData(std::vector<double>& time_since_skill_started_vector,
                                 std::vector<std::array<double, 16>>& pose_desired_vector,
                                 std::vector<std::array<double, 16>>& O_T_EE_vector,
                                 std::vector<std::array<double, 16>>& O_T_EE_d_vector,
                                 std::vector<std::array<double, 16>>& F_T_EE_vector,
                                 std::vector<std::array<double, 16>>& EE_T_K_vector,
                                 std::vector<double>& m_ee_vector,
                                 std::vector<std::array<double, 9>>& I_ee_vector,
                                 std::vector<std::array<double, 3>>& F_x_Cee_vector,
                                 std::vector<double>& m_load_vector,
                                 std::vector<std::array<double, 9>>& I_load_vector,
                                 std::vector<std::array<double, 3>>& F_x_Cload_vector,
                                 std::vector<double>& m_total_vector,
                                 std::vector<std::array<double, 9>>& I_total_vector,
                                 std::vector<std::array<double, 3>>& F_x_Ctotal_vector,
                                 std::vector<std::array<double, 2>>& elbow_vector,
                                 std::vector<std::array<double, 2>>& elbow_d_vector,
                                 std::vector<std::array<double, 2>>& elbow_c_vector,
                                 std::vector<std::array<double, 2>>& delbow_c_vector,
                                 std::vector<std::array<double, 2>>& ddelbow_c_vector,
                                 std::vector<std::array<double, 7>>& tau_J_vector,
                                 std::vector<std::array<double, 7>>& tau_J_d_vector,
                                 std::vector<std::array<double, 7>>& dtau_J_vector,
                                 std::vector<std::array<double, 7>>& q_vector,
                                 std::vector<std::array<double, 7>>& q_d_vector,
                                 std::vector<std::array<double, 7>>& dq_vector,
                                 std::vector<std::array<double, 7>>& dq_d_vector,
                                 std::vector<std::array<double, 7>>& ddq_d_vector,
                                 std::vector<std::array<double, 7>>& joint_contact_vector,
                                 std::vector<std::array<double, 6>>& cartesian_contact_vector,
                                 std::vector<std::array<double, 7>>& joint_collision_vector,
                                 std::vector<std::array<double, 6>>& cartesian_collision_vector,
                                 std::vector<std::array<double, 7>>& tau_ext_hat_filtered_vector,
                                 std::vector<std::array<double, 6>>& O_F_ext_hat_K_vector,
                                 std::vector<std::array<double, 6>>& K_F_ext_hat_K_vector,
                                 std::vector<std::array<double, 6>>& O_dP_EE_d_vector,
                                 std::vector<std::array<double, 16>>& O_T_EE_c_vector,
                                 std::vector<std::array<double, 6>>& O_dP_EE_c_vector,
                                 std::vector<std::array<double, 6>>& O_ddP_EE_c_vector,
                                 std::vector<std::array<double, 7>>& theta_vector,
                                 std::vector<std::array<double, 7>>& dtheta_vector,
                                 std::vector<std::array<bool, 37>>& current_errors_vector,
                                 std::vector<std::array<bool, 37>>& last_motion_errors_vector,
                                 std::vector<double>& control_command_success_rate_vector,
                                 std::vector<uint8_t>& robot_mode_vector,
                                 std::vector<double>& robot_time_vector,
                                 std::vector<double>& gripper_width_vector,
                                 std::vector<double>& gripper_max_width_vector,
                                 std::vector<bool>& gripper_is_grasped_vector,
                                 std::vector<uint16_t>& gripper_temperature_vector,
                                 std::vector<double>& gripper_time_vector) {
    if (!open_file_stream_.is_open()) {
        open_file_stream_ = std::ofstream(filename_, std::ofstream::out | std::ofstream::app);
    }

    bool all_sizes_equal = true;
    size_t time_since_skill_started_vector_size = time_since_skill_started_vector.size();
    if (write_pose_desired_ && time_since_skill_started_vector_size != pose_desired_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and pose desired vector size do not match\n";
    } else if (time_since_skill_started_vector_size != O_T_EE_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and O_T_EE vector size do not match\n";
    } else if (time_since_skill_started_vector_size != O_T_EE_d_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and O_T_EE_d vector size do not match\n";
    } else if (time_since_skill_started_vector_size != F_T_EE_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and F_T_EE vector size do not match\n";
    } else if (time_since_skill_started_vector_size != EE_T_K_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and EE_T_K vector size do not match\n";
    } else if (time_since_skill_started_vector_size != m_ee_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and m_ee vector size do not match\n";
    } else if (time_since_skill_started_vector_size != I_ee_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and I_ee vector size do not match\n";
    } else if (time_since_skill_started_vector_size != F_x_Cee_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and F_x_Cee vector size do not match\n";
    } else if (time_since_skill_started_vector_size != m_load_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and m_load vector size do not match\n";
    } else if (time_since_skill_started_vector_size != I_load_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and I_load vector size do not match\n";
    } else if (time_since_skill_started_vector_size != F_x_Cload_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and F_x_Cload vector size do not match\n";
    } else if (time_since_skill_started_vector_size != m_total_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and m_total vector size do not match\n";
    } else if (time_since_skill_started_vector_size != I_total_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and I_total vector size do not match\n";
    } else if (time_since_skill_started_vector_size != F_x_Ctotal_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and F_x_Ctotal vector size do not match\n";
    } else if (time_since_skill_started_vector_size != elbow_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and elbow vector size do not match\n";
    } else if (time_since_skill_started_vector_size != elbow_d_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and elbow_d vector size do not match\n";
    } else if (time_since_skill_started_vector_size != elbow_c_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and elbow_c vector size do not match\n";
    } else if (time_since_skill_started_vector_size != delbow_c_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and delbow_c vector size do not match\n";
    } else if (time_since_skill_started_vector_size != ddelbow_c_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and ddelbow_c vector size do not match\n";
    } else if (time_since_skill_started_vector_size != tau_J_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and tau_J vector size do not match\n";
    } else if (time_since_skill_started_vector_size != tau_J_d_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and tau_J_d vector size do not match\n";
    } else if (time_since_skill_started_vector_size != dtau_J_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and dtau_J vector size do not match\n";
    } else if (time_since_skill_started_vector_size != q_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and q vector size do not match\n";
    } else if (time_since_skill_started_vector_size != q_d_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and q_d vector size do not match\n";
    } else if (time_since_skill_started_vector_size != dq_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and dq vector size do not match\n";
    } else if (time_since_skill_started_vector_size != dq_d_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and dq_d vector size do not match\n";
    } else if (time_since_skill_started_vector_size != ddq_d_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and ddq_d vector size do not match\n";
    } else if (time_since_skill_started_vector_size != joint_contact_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and joint_contact vector size do not match\n";
    } else if (time_since_skill_started_vector_size != cartesian_contact_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and cartesian_contact vector size do not match\n";
    } else if (time_since_skill_started_vector_size != joint_collision_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and joint_collision vector size do not match\n";
    } else if (time_since_skill_started_vector_size != cartesian_collision_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and cartesian_collision vector size do not match\n";
    } else if (time_since_skill_started_vector_size != tau_ext_hat_filtered_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and tau_ext_hat_filtered vector size do not match\n";
    } else if (time_since_skill_started_vector_size != O_F_ext_hat_K_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and O_F_ext_hat_K vector size do not match\n";
    } else if (time_since_skill_started_vector_size != K_F_ext_hat_K_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and K_F_ext_hat_K vector size do not match\n";
    } else if (time_since_skill_started_vector_size != O_dP_EE_d_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and O_dP_EE_d vector size do not match\n";
    } else if (time_since_skill_started_vector_size != O_T_EE_c_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and O_T_EE_c vector size do not match\n";
    } else if (time_since_skill_started_vector_size != O_dP_EE_c_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and O_dP_EE_c vector size do not match\n";
    } else if (time_since_skill_started_vector_size != O_ddP_EE_c_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and O_ddP_EE_c vector size do not match\n";
    } else if (time_since_skill_started_vector_size != theta_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and theta vector size do not match\n";
    } else if (time_since_skill_started_vector_size != dtheta_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and dtheta vector size do not match\n";
    } else if (time_since_skill_started_vector_size != current_errors_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and current errors vector size do not match\n";
    } else if (time_since_skill_started_vector_size != last_motion_errors_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and last motion errors vector size do not match\n";
    } else if (time_since_skill_started_vector_size != control_command_success_rate_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and control command success rate vector size do not match\n";
    } else if (time_since_skill_started_vector_size != robot_mode_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and robot_mode vector size do not match\n";
    } else if (time_since_skill_started_vector_size != robot_time_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and robot time vector size do not match\n";
    } else if (time_since_skill_started_vector_size != gripper_width_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and gripper width vector size do not match\n";
    } else if (time_since_skill_started_vector_size != gripper_max_width_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and gripper max width vector size do not match\n";
    } else if (time_since_skill_started_vector_size != gripper_is_grasped_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and gripper is grasped vector size do not match\n";
    } else if (time_since_skill_started_vector_size != gripper_temperature_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and gripper temperature vector size do not match\n";
    } else if (time_since_skill_started_vector_size != gripper_time_vector.size()) {
        all_sizes_equal = false;
        std::cout << "Time since skill started vector size and gripper time vector size do not match\n";
    }

    if (!all_sizes_equal) {
        std::cout << "Save vectors do not all have the same size. Will not save!!!" << std::endl;
        // Can throw error
        return false;
    }

    for (int i = 0; i < time_since_skill_started_vector_size; i++) {
        open_file_stream_ << time_since_skill_started_vector[i] << ",";

        if (write_pose_desired_) {
            std::array<double, 16> &pose_desired = pose_desired_vector[i];
            for (const auto &e : pose_desired) {
                open_file_stream_ << e << ",";
            }
        }

        std::array<double, 16>& O_T_EE = O_T_EE_vector[i];
        for (const auto &e : O_T_EE) {
            open_file_stream_ << e << ",";
        }
        
        std::array<double, 16>& O_T_EE_d = O_T_EE_d_vector[i];
        for (const auto &e : O_T_EE_d) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 16>& F_T_EE = F_T_EE_vector[i];
        for (const auto &e : F_T_EE) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 16>& EE_T_K = EE_T_K_vector[i];
        for (const auto &e : EE_T_K) {
            open_file_stream_ << e << ",";
        }

        open_file_stream_ << m_ee_vector[i] << ",";

        std::array<double, 9>& I_ee = I_ee_vector[i];
        for (const auto &e : I_ee) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 3>& F_x_Cee = F_x_Cee_vector[i];
        for (const auto &e : F_x_Cee) {
            open_file_stream_ << e << ",";
        }

        open_file_stream_ << m_load_vector[i] << ",";

        std::array<double, 9>& I_load = I_load_vector[i];
        for (const auto &e : I_load) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 3>& F_x_Cload = F_x_Cload_vector[i];
        for (const auto &e : F_x_Cload) {
            open_file_stream_ << e << ",";
        }

        open_file_stream_ << m_total_vector[i] << ",";

        std::array<double, 9>& I_total = I_total_vector[i];
        for (const auto &e : I_total) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 3>& F_x_Ctotal = F_x_Ctotal_vector[i];
        for (const auto &e : F_x_Ctotal) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 2>& elbow = elbow_vector[i];
        for (const auto &e : elbow) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 2>& elbow_d = elbow_d_vector[i];
        for (const auto &e : elbow_d) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 2>& elbow_c = elbow_c_vector[i];
        for (const auto &e : elbow_c) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 2>& delbow_c = delbow_c_vector[i];
        for (const auto &e : delbow_c) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 2>& ddelbow_c = ddelbow_c_vector[i];
        for (const auto &e : ddelbow_c) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 7>& tau_J = tau_J_vector[i];
        for (const auto &e : tau_J) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 7>& tau_J_d = tau_J_d_vector[i];
        for (const auto &e : tau_J_d) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 7>& dtau_J = dtau_J_vector[i];
        for (const auto &e : dtau_J) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 7>& q = q_vector[i];
        for (const auto &e : q) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 7>& q_d = q_d_vector[i];
        for (const auto &e : q_d) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 7>& dq = dq_vector[i];
        for (const auto &e : dq) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 7>& dq_d = dq_d_vector[i];
        for (const auto &e : dq_d) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 7>& ddq_d = ddq_d_vector[i];
        for (const auto &e : ddq_d) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 7>& joint_contact = joint_contact_vector[i];
        for (const auto &e : joint_contact) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 6>& cartesian_contact = cartesian_contact_vector[i];
        for (const auto &e : cartesian_contact) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 7>& joint_collision = joint_collision_vector[i];
        for (const auto &e : joint_collision) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 6>& cartesian_collision = cartesian_collision_vector[i];
        for (const auto &e : cartesian_collision) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 7>& tau_ext_hat_filtered = tau_ext_hat_filtered_vector[i];
        for (const auto &e : tau_ext_hat_filtered) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 6>& O_F_ext_hat_K = O_F_ext_hat_K_vector[i];
        for (const auto &e : O_F_ext_hat_K) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 6>& K_F_ext_hat_K = K_F_ext_hat_K_vector[i];
        for (const auto &e : K_F_ext_hat_K) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 6>& O_dP_EE_d = O_dP_EE_d_vector[i];
        for (const auto &e : O_dP_EE_d) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 16>& O_T_EE_c = O_T_EE_c_vector[i];
        for (const auto &e : O_T_EE_c) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 6>& O_dP_EE_c = O_dP_EE_c_vector[i];
        for (const auto &e : O_dP_EE_c) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 6>& O_ddP_EE_c = O_ddP_EE_c_vector[i];
        for (const auto &e : O_ddP_EE_c) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 7>& theta = theta_vector[i];
        for (const auto &e : theta) {
            open_file_stream_ << e << ",";
        }

        std::array<double, 7>& dtheta = dtheta_vector[i];
        for (const auto &e : dtheta) {
            open_file_stream_ << e << ",";
        }

        std::array<bool, 37>& current_errors = current_errors_vector[i];
        for (const auto &e : current_errors) {
            open_file_stream_ << e << ",";
        }

        std::array<bool, 37>& last_motion_errors = last_motion_errors_vector[i];
        for (const auto &e : last_motion_errors) {
            open_file_stream_ << e << ",";
        }

        open_file_stream_ << control_command_success_rate_vector[i] << ",";

        open_file_stream_ << robot_mode_vector[i] << ",";

        open_file_stream_ << robot_time_vector[i] << ",";

        open_file_stream_ << gripper_width_vector[i] << ",";

        open_file_stream_ << gripper_max_width_vector[i] << ",";

        open_file_stream_ << gripper_is_grasped_vector[i] << ",";

        open_file_stream_ << gripper_temperature_vector[i] << ",";

        open_file_stream_ << gripper_time_vector[i];

        open_file_stream_ << "\n";
    }
    open_file_stream_.close();
    return true;
}

bool FileStreamLogger::writeStringData(std::vector<std::string> data) {
    if (!open_file_stream_.is_open()) {
        open_file_stream_ = std::ofstream(filename_, std::ofstream::out | std::ofstream::app);
    }
    size_t data_size = data.size();
    for (int i = 0; i < data_size ; i++) {
        open_file_stream_ << "info: " << data[i] << "\n";
    }
}
