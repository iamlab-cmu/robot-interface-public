#pragma once

#include <array>
#include <fstream>
#include <vector>

class FileStreamLogger {
 public:
   FileStreamLogger(): open_file_stream_("./traj_data.txt", std::ofstream::out | std::ofstream::app) {};

   bool writeData(std::vector<double> control_time,
                  std::vector<std::array<double, 16>>& pose_desired,
                  std::vector<std::array<double, 16>>& robot_state,
                  std::vector<std::array<double, 7>>& tau_j,
                  std::vector<std::array<double, 7>>& d_tau_j,
                  std::vector<std::array<double, 7>>& q,
                  std::vector<std::array<double, 7>>& q_d,
                  std::vector<std::array<double, 7>>& dq);

 private:
  std::ofstream open_file_stream_;
};
