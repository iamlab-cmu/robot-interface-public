//
// Created by mohit on 11/25/18.
//

#include "iam_robolib/termination_handler/termination_handler.h"
#include <Eigen/Dense>
#include <vector>
#include <array>

void TerminationHandler::check_terminate_preempt() {
  if (!done_) {
    done_ = run_loop_info_->get_skill_preempted();
  }
};

bool TerminationHandler::has_terminated() {
  return done_;
}

void TerminationHandler::check_terminate_virtual_wall_collisions(const franka::RobotState &robot_state, franka::Model *robot_model) {
  if (!done_) {

    // Create hyperplanes
    std::vector<Eigen::Hyperplane<double,3>> planes;
    planes.push_back(
      Eigen::Hyperplane<double,3>(Eigen::Vector3d(1., 0., 0.), Eigen::Vector3d(0.5, 0., 0.))
    );

    // Create dist thresholds
    std::array<double, 9> dist_thresholds = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

    int n_frame = 0;
    for (franka::Frame frame = franka::Frame::kJoint1; frame <= franka::Frame::kEndEffector; frame++) 
    {
      std::array<double, 16> pose = robot_model->pose(frame, robot_state);
      Eigen::Vector3d pos(pose[12], pose[13], pose[14]);
    
      for (uint n_plane = 0; n_plane < planes.size(); n_plane++)
      {
        double dist = planes[n_plane].absDistance(pos);
        if (dist < dist_thresholds[n_frame++])
        {
          printf("Frame %d is in collision with wall %d with distance %f\n", n_frame, n_plane, dist);
          done_ = true;
          break;
        }
      }

      if (done_) break;
    }
  }
}