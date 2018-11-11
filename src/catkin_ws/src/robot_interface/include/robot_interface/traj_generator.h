#pragma once

#include <vector>
#include <memory>
#include <string>

#include <ros/node_handle.h>
#include <ros/time.h>

// TODO(Mohit): Add a namespace to these declarations.
// TODO(Mohit): Need to make this an interface.
class TrajectoryGenerator {
 public:
  // TODO(Mohit): Maybe we should pass in a pointer to the main loop interface?
  bool init(vector<float> &trajectory_params, ros::NodeHandle& node_handle);
  void starting(const ros::Time&);
  void update(const ros::Time&, const ros::Duration& period);

 private:
  ros::Duration elapsed_time_;
  // std::array<double> initial_sequence{};
  std::vector<int> generated_sequence{};

  // Counting trajectory generator (This needs to be refined)
  int _start_value=0;
  int _current_value=0;
  int _delta_value=0;
};
