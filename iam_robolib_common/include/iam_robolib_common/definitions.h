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
    SaveTrajectorySkill = 3,
    ForceTorqueSkill = 4
};

// Enum for Meta Skill Types
enum class MetaSkillType : uint8_t {
    BaseMetaSkill = 0,
    JointPoseContinuousSkill = 1
};

// Enum for Trajectory Generator Types
enum class TrajectoryGeneratorType : uint8_t {
    CounterTrajectoryGenerator = 1,
    LinearTrajectoryGenerator = 2,
    LinearJointTrajectoryGenerator = 3,
    LinearTrajectoryGeneratorWithTimeAndGoal = 4,
    GripperTrajectoryGenerator = 5,
    StayInInitialPositionTrajectoryGenerator = 6,
    DmpTrajectoryGenerator = 7,
    RelativeLinearTrajectoryGenerator = 8,
    ImpulseTrajectoryGenerator = 9,
    MinJerkJointTrajectoryGenerator = 10
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
    NoopTerminationHandler = 1,
    FinalPoseTerminationHandler = 2,
    FinalJointTerminationHandler = 3,
    LinearTrajectoryGeneratorWithTimeAndGoalTerminationHandler = 4,
    ContactTerminationHandler = 5,
    TimeTerminationHandler = 6
};

// Enum for Skill Statuses
enum class SkillStatus : uint8_t { 
  TO_START = 0, 
  RUNNING = 1, 
  FINISHED = 2 
}; 

#endif  // IAM_ROBOLIB_COMMON_DEFINITIONS_H_