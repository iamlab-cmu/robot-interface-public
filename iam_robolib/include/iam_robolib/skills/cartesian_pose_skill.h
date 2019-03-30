#ifndef IAM_ROBOLIB_SKILLS_CARTESIAN_POSE_SKILL_H_
#define IAM_ROBOLIB_SKILLS_CARTESIAN_POSE_SKILL_H_

#include "iam_robolib/skills/base_skill.h"

#include "iam_robolib/trajectory_generator/trajectory_generator.h"
#include "iam_robolib/feedback_controller/feedback_controller.h"
#include "iam_robolib/termination_handler/termination_handler.h"

class CartesianPoseSkill : public BaseSkill {
 public:
  CartesianPoseSkill(int skill_idx, int meta_skill_idx, std::string description) : 
                              BaseSkill(skill_idx, meta_skill_idx, description)
  {};

  void execute_skill() override;

  void execute_skill_on_franka(run_loop* run_loop,
                               FrankaRobot* robot,
                               RobotStateData* robot_state_data) override;

 private:
  bool return_status_{false};
};

#endif  // IAM_ROBOLIB_SKILLS_CARTESIAN_POSE_SKILL_H_