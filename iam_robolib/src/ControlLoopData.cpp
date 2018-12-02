#include "ControlLoopData.h"

#include <iostream>

#include <iam_robolib/run_loop.h>

std::atomic<bool> ControlLoopData::use_buffer_0{true};
std::mutex ControlLoopData::buffer_0_mutex_;
std::mutex ControlLoopData::buffer_1_mutex_;

template<int N>
void printListOfVectors(std::vector<std::array<double, N>> data, int print_last) {
  int print_counter = 0;
  for (auto it = data.rbegin(); it != data.rend(); it++, print_counter++) {
    std::array<double, N> temp = *it;
    for (size_t j = 0; j < temp.size(); j++) {
      std::cout << temp[j] << " ";
    }
    std::cout << "\n" << std::endl;
    if (++print_counter < print_last) {
      break;
    }
  }
}

void ControlLoopData::printMeasuredJointJerks(std::vector<std::array<double, 7>> data, int print_last) {
  int print_counter = 0;
  for (auto it = data.rbegin(); it != data.rend(); it++) {
    std::array<double, 7> temp = *it;
    std::array<double, 7> temp2 = *(it + 1);
    std::array<double, 7> temp3 = *(it + 2);

    for (size_t j = 0; j < temp.size(); j++) {
      std::cout << (((temp3[j] - temp2[j]) * 1000) - ((temp2[j] - temp[j]) * 1000)) * 1000 << " ";
    }
    std::cout << "\n" << std::endl;
    print_last = print_last - 1;
    if (++print_counter < print_last) {
      break;
    }
  }
}

void ControlLoopData::writeBufferData_0() {
    std::lock_guard<std::mutex> lock(buffer_0_mutex_);
    std::cout << "Will save buffer 0\n";

    log_pose_desired_g_.insert(
        log_pose_desired_g_.end(), log_pose_desired_0_.begin(), log_pose_desired_0_.end());
    log_robot_state_g_.insert(
        log_robot_state_g_.end(), log_robot_state_0_.begin(), log_robot_state_0_.end());
    log_tau_j_g_.insert(log_tau_j_0_.end(), log_tau_j_0_.begin(), log_tau_j_0_.end());
    log_dq_g_.insert(log_dq_g_.end(), log_dq_0_.begin(), log_dq_0_.end());
    log_control_time_g_.insert(
        log_control_time_g_.end(), log_control_time_0_.begin(), log_control_time_0_.end());

    // For now just clear them
    log_pose_desired_0_.clear();
    log_robot_state_0_.clear();
    log_tau_j_0_.clear();
    log_dq_0_.clear();
    log_control_time_0_.clear();

    std::cout << "Did save buffer 0\n";
};

void ControlLoopData::writeBufferData_1() {
    std::lock_guard<std::mutex> lock(buffer_1_mutex_);
    std::cout << "Will save buffer 1\n";

    log_pose_desired_g_.insert(
        log_pose_desired_g_.end(), log_pose_desired_1_.begin(), log_pose_desired_1_.end());
    log_robot_state_g_.insert(
        log_robot_state_g_.end(), log_robot_state_1_.begin(), log_robot_state_1_.end());
    log_tau_j_g_.insert(log_tau_j_0_.end(), log_tau_j_1_.begin(), log_tau_j_1_.end());
    log_dq_g_.insert(log_dq_g_.end(), log_dq_1_.begin(), log_dq_1_.end());
    log_control_time_g_.insert(
        log_control_time_g_.end(), log_control_time_1_.begin(), log_control_time_1_.end());

    log_pose_desired_1_.clear();
    log_robot_state_1_.clear();
    log_tau_j_1_.clear();
    log_dq_1_.clear();
    log_control_time_1_.clear();

    std::cout << "Did save buffer 1\n";
};

void ControlLoopData::startFileLoggerThread() {
    file_logger_thread_ = std::thread([&]() {
      // Sleep to achieve the desired print rate.
      while (RunLoop::running_skills_) {
          std::this_thread::sleep_for(
              std::chrono::milliseconds(static_cast<int>((1.0 / log_rate_ * 1000.0))));
          // Try to lock data to avoid read write collisions.
          bool did_write_to_buffer_0 = false;

          // Control loop thread should now switch between buffers.
          if (use_buffer_0) {
            use_buffer_0 = false;
            did_write_to_buffer_0 = true;
          } else {
            use_buffer_0 = true;
          }
          if (did_write_to_buffer_0) {
            writeBufferData_0();
          } else {
            writeBufferData_1();
          }
      }
    });
}

void ControlLoopData::writeCurrentBufferData() {
    if (use_buffer_0) {
      writeBufferData_0();
    } else {
      writeBufferData_1();
    }
}

void ControlLoopData::printGlobalData(int print_count) {
  std::cout << "===== Robots state ======\n";
  printListOfVectors<16>(log_robot_state_g_, print_count);

  std::cout << "===== Desired Pose ======\n";
  printListOfVectors<16>(log_pose_desired_g_, print_count);

  std::cout << "===== Measured link-side joint torque sensor signals ======\n";
  printListOfVectors<7>(log_tau_j_g_, print_count);

  std::cout << "===== Measured joint velocity ======\n";
  printListOfVectors<7>(log_dq_g_, print_count);

  std::cout << "===== Measured joint jerks ======\n";
  printMeasuredJointJerks(log_dq_g_, print_count);
}
