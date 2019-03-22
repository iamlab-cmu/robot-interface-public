#######################################################################
#                                                                     #
#   Important: Any Changes here should also be reflected in changes   #
#   in the frankpy iam_robolib_common_definitions.py file as well.    #
#                                                                     #
####################################################################### 

class RobotType:
    FRANKA = 0
    UR5E = 1

class SkillType:
    SkillInfo = 0
    GripperSkill = 1
    JointPoseSkill = 2
    ForceTorqueSkill = 3

class MetaSkillType:
    BaseMetaSkill = 0
    JointPoseContinuousSkill = 1

class TrajectoryGeneratorType:
    LinearPoseTrajectoryGenerator = 1
    LinearJointTrajectoryGenerator = 2
    GripperTrajectoryGenerator = 3
    StayInInitialPositionTrajectoryGenerator = 4
    DmpTrajectoryGenerator = 5
    RelativeLinearPoseTrajectoryGenerator = 6
    ImpulseTrajectoryGenerator = 7
    MinJerkJointTrajectoryGenerator = 8

class FeedbackControllerType:
    NoopFeedbackController = 1
    TorqueFeedbackController = 2
    CustomGainTorqueController = 3
    ForceAxisImpedenceFeedbackController = 4
    PassThroughFeedbackController = 5

class TerminationHandlerType:
    ContactTerminationHandler = 1
    FinalJointTerminationHandler = 2
    FinalPoseTerminationHandler = 3
    NoopTerminationHandler = 4
    TimeTerminationHandler = 5

class SkillStatus: 
    TO_START = 0
    RUNNING = 1
    FINISHED = 2 