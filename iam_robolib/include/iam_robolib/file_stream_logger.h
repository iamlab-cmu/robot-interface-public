#ifndef IAM_ROBOLIB_FILE_STREAM_LOGGER_H_
#define IAM_ROBOLIB_FILE_STREAM_LOGGER_H_

#include <array>
#include <fstream>
#include <vector>

class FileStreamLogger {
 public:
  FileStreamLogger(const std::string& filename): filename_(filename),
                                                 open_file_stream_(filename, std::ofstream::out | std::ofstream::app) {};

  bool write_pose_desired_=true;

  bool writeData(std::vector<double>& time_since_skill_started_vector,
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
                 double gripper_width,
                 double gripper_max_width,
                 bool gripper_is_grasped,
                 uint16_t gripper_temperature,
                 double gripper_time);

  /**
   * Write string data to logger. String data is prefixed by "info:" to allow us to easily find it in the logs.
   * @param data String data to log does not require a "\n" in the end.
   * @return True if we did write the data successfully else false.
   */
  bool writeStringData(std::vector<std::string> data);

 private:
  std::ofstream open_file_stream_;
  std::string filename_;
};

#endif  // IAM_ROBOLIB_FILE_STREAM_LOGGER_H_