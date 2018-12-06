//
// Created by mohit on 12/5/18.
//

#include "FileStreamLogger.h"

#include <iostream>

bool FileStreamLogger::writeData(std::vector<double> control_time,
               std::vector<std::array<double, 16>>& pose_desired,
               std::vector<std::array<double, 16>>& robot_state,
               std::vector<std::array<double, 7>>& tau_j,
               std::vector<std::array<double, 7>>& dq) {
    return true;
    if (!open_file_stream_.is_open()) {
        open_file_stream_ = std::ofstream("./data/traj_data.txt", std::ofstream::out | std::ofstream::app);
    }

    bool all_sizes_equal = true;
    size_t control_time_size = control_time.size();
    if (control_time_size != pose_desired.size()) {
        all_sizes_equal = false;
        std::cout << "Control time and pose desired size do not match\n";
    } else if (control_time_size != robot_state.size()) {
        all_sizes_equal = false;
        std::cout << "Control time and robot state size do not match\n";
    } else if (control_time_size != tau_j.size()) {
        all_sizes_equal = false;
        std::cout << "Control time and tau_j size do not match\n";
    } else if (control_time_size != dq.size()) {
        all_sizes_equal = false;
        std::cout << "Control time and dq_ size do not match\n";
    }

    if (!all_sizes_equal) {
        std::cout << "Save vectors do not have same size. Will not save!!!" << std::endl;
        // Can throw error
        return false;
    }

    for (int i = 0; i < control_time_size; i++) {
        open_file_stream_ << control_time[i] << ",";
        std::array<double, 16> &pose = pose_desired[i];
        for (const auto &e : pose) {
            open_file_stream_ << e << ",";
        }
        pose = robot_state[i];
        for (const auto &e : pose) {
            open_file_stream_ << e << ",";
        }
        std::array<double, 7> &state = tau_j[i];
        for (const auto &e : state) {
            open_file_stream_ << e << ",";
        }
        state = dq[i];
        for (const auto &e : state) {
            open_file_stream_ << e << ",";
        }
        open_file_stream_ << "\n";
    }
    open_file_stream_.close();
    return true;
}
