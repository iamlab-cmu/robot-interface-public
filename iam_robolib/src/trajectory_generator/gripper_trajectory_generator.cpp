//
// Created by iamlab on 12/1/18.
//

#include "iam_robolib/trajectory_generator/gripper_trajectory_generator.h"

#include <iostream>

void GripperTrajectoryGenerator::parse_parameters() {
  int num_params = static_cast<int>(params_[1]);

  if(num_params == 2) {
    width_ = static_cast<double >(params_[2]);
    speed_ = static_cast<double >(params_[3]);
    is_grasp_skill_ = false;
  } else if (num_params == 3) {
    width_ = static_cast<double >(params_[2]);
    speed_ = static_cast<double >(params_[3]);
    force_ = static_cast<double> (params_[4]);
    is_grasp_skill_ = true;
  } else {
    std::cout << "GripperTrajGen: Incorrect number of params given: " << num_params << std::endl;
  }
}

void GripperTrajectoryGenerator::initialize_trajectory() {
  // pass
}

void GripperTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state) {
  // pass
}

void GripperTrajectoryGenerator::get_next_step() {
  // pass
}

double GripperTrajectoryGenerator::getWidth() {
  return width_;
}

double GripperTrajectoryGenerator::getSpeed()  {
  return speed_;
}

double GripperTrajectoryGenerator::getForce()  {
  return force_;
}

bool GripperTrajectoryGenerator::isGraspSkill(){
  return is_grasp_skill_;
}
