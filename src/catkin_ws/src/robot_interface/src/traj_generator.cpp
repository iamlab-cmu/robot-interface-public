#include <robot_interfae/traj_generator.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <ros/ros.h>

bool TrajectoryGenerator::init(vector<float> &trajectory_params, 
                               ros::NodeHandle& node_handle) {

  _start_value = trajectory_params[0];
  _delta_value = trajectory_params[1];

  return true;
}

void TrajectoryGenerator::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  elapsed_time_ = ros::Duration(0.0);
}

void TrajectoryGenerator::update(const ros::Time& /* time */,
                                 const ros::Duration& period) {
  elapsed_time_ += period;
  _current_value += _delta_value;
  generated_sequence.push_back(_current_value)
}
