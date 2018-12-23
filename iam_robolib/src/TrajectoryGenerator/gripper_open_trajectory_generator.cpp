//
// Created by iamlab on 12/1/18.
//

#include "gripper_open_trajectory_generator.h"

#include <iostream>

void gripper_open_trajectory_generator::parse_parameters() {
  int num_params = static_cast<int>(params_[1]);

  if(num_params == 3) {
    width_ = static_cast<double >(params_[2]);
    speed_ = static_cast<double >(params_[3]);
    wait_time_in_milliseconds_ = static_cast<double>(params_[4]);
    is_grasp_skill_ = false;
  } else if (num_params == 4) {
    width_ = static_cast<double >(params_[2]);
    speed_ = static_cast<double >(params_[3]);
    force_ = static_cast<double> (params_[4]);
    wait_time_in_milliseconds_ = static_cast<double>(params_[5]);
    is_grasp_skill_ = true;
  } else {
    std::cout << "GripperOpenTrajGen: Incorrect number of params given: " << num_params << std::endl;
  }
}

void gripper_open_trajectory_generator::initialize_trajectory() {
  // pass
}

void gripper_open_trajectory_generator::initialize_trajectory(const franka::RobotState &robot_state) {
  // pass
}

void gripper_open_trajectory_generator::get_next_step() {
  // pass
}

double gripper_open_trajectory_generator::getWidth() {
  return width_;
}

double gripper_open_trajectory_generator::getSpeed()  {
  return speed_;
}

double gripper_open_trajectory_generator::getForce()  {
  return force_;
}

bool gripper_open_trajectory_generator::isGraspSkill(){
  return is_grasp_skill_;
}

double gripper_open_trajectory_generator::getWaitTimeInMilliseconds() {
  return wait_time_in_milliseconds_;
}
