#ifndef IAM_ROBOLIB_FEEDBACK_CONTROLLER_SET_INTERNAL_IMPEDANCE_FEEDBACK_CONTROLLER_H_
#define IAM_ROBOLIB_FEEDBACK_CONTROLLER_SET_INTERNAL_IMPEDANCE_FEEDBACK_CONTROLLER_H_

#include "iam_robolib/feedback_controller/feedback_controller.h"

class SetInternalImpedanceFeedbackController : public FeedbackController {
 public:
  using FeedbackController::FeedbackController;

  void parse_parameters() override;

  void initialize_controller() override;

  void initialize_controller(FrankaRobot *robot) override;

  void get_next_step() override;

  void get_next_step(const franka::RobotState &robot_state, 
                     TrajectoryGenerator *traj_generator) override;

  bool set_joint_impedance_ = false;
  bool set_cartesian_impedance_ = false;
  std::array<double, 7> K_theta_ = {{3000, 3000, 3000, 2500, 2500, 2000, 2000}};
  std::array<double, 6> K_x_ = {{3000, 3000, 3000, 300, 300, 300}};
};

#endif  // IAM_ROBOLIB_FEEDBACK_CONTROLLER_SET_INTERNAL_IMPEDANCE_FEEDBACK_CONTROLLER_H_