#ifndef IAM_ROBOLIB_FEEDBACK_CONTROLLER_PASSTHROUGH_FEEDBACK_CONTROLLER_H_
#define IAM_ROBOLIB_FEEDBACK_CONTROLLER_PASSTHROUGH_FEEDBACK_CONTROLLER_H_

#include <Eigen/Dense>

#include "iam_robolib/feedback_controller/feedback_controller.h"

// A passthrough feedback controller that just usese the desired force 
// torque of traj gen and passes it to the robot joint torques through 
// the jacobian
class PassThroughFeedbackController : public FeedbackController {
 public:
  using FeedbackController::FeedbackController;

  void parse_parameters() override;

  void initialize_controller() override;

  void initialize_controller(FrankaRobot *robot) override;

  void get_next_step() override;

  void get_next_step(const franka::RobotState &robot_state, 
                     TrajectoryGenerator *traj_generator) override;

 private:
  const franka::Model *model_;

};

#endif  // IAM_ROBOLIB_FEEDBACK_CONTROLLER_PASSTHROUGH_FEEDBACK_CONTROLLER_H_