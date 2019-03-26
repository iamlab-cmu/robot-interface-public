#ifndef IAM_ROBOLIB_COMMON_DEFINITIONS_H_
#define IAM_ROBOLIB_COMMON_DEFINITIONS_H_

#include <stdint.h>

/*
 *
 *  Important: Any Changes here should also be reflected in changes
 *  in the frankpy iam_robolib_common_definitions.py file as well.
 *
 */

// SharedBuffer type to share memory (Change size later)
typedef double SharedBufferType;
typedef SharedBufferType* SharedBufferTypePtr;

// Enum for Robot Types
enum class RobotType : uint8_t {
    FRANKA = 0,
    UR5E = 1
};

// Enum for Skill Types
enum class SkillType : uint8_t {
    SkillInfo = 0,
    GripperSkill = 1,
    JointPoseSkill = 2,
    ForceTorqueSkill = 3
};

// Enum for Meta Skill Types
enum class MetaSkillType : uint8_t {
    BaseMetaSkill = 0,
    JointPoseContinuousSkill = 1
};

// Enum for Trajectory Generator Types
enum class TrajectoryGeneratorType : uint8_t {
    LinearPoseTrajectoryGenerator = 1,
    LinearJointTrajectoryGenerator = 2,
    GripperTrajectoryGenerator = 3,
    StayInInitialPositionTrajectoryGenerator = 4,
    JointDmpTrajectoryGenerator = 5,
    RelativeLinearPoseTrajectoryGenerator = 6,
    ImpulseTrajectoryGenerator = 7,
    MinJerkJointTrajectoryGenerator = 8
};

// Enum for Feedback Controller Types
enum class FeedbackControllerType : uint8_t {
    NoopFeedbackController = 1,
    TorqueFeedbackController = 2,
    CustomGainTorqueController = 3,
    ForceAxisImpedenceFeedbackController = 4,
    PassThroughFeedbackController = 5
};

// Enum for Termination Handler Types
enum class TerminationHandlerType : uint8_t {
    ContactTerminationHandler = 1,
    FinalJointTerminationHandler = 2,
    FinalPoseTerminationHandler = 3,
    NoopTerminationHandler = 4,
    TimeTerminationHandler = 5
};

// Enum for Skill Statuses
enum class SkillStatus : uint8_t { 
  TO_START = 0, 
  RUNNING = 1, 
  FINISHED = 2 
}; 

#endif  // IAM_ROBOLIB_COMMON_DEFINITIONS_H_