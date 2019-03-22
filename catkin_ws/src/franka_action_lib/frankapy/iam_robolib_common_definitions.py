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
    SaveTrajectorySkill = 3
    ForceTorqueSkill = 4

class MetaSkillType:
    BaseMetaSkill = 0
    JointPoseContinuousSkill = 1

class TrajectoryGeneratorType:
    CounterTrajectoryGenerator = 1
    LinearTrajectoryGenerator = 2
    LinearJointTrajectoryGenerator = 3
    LinearTrajectoryGeneratorWithTimeAndGoal = 4
    GripperTrajectoryGenerator = 5
    StayInInitialPositionTrajectoryGenerator = 6
    DmpTrajectoryGenerator = 7
    RelativeLinearTrajectoryGenerator = 8
    ImpulseTrajectoryGenerator = 9
    MinJerkJointTrajectoryGenerator = 10

class FeedbackControllerType:
    NoopFeedbackController = 1
    TorqueFeedbackController = 2
    CustomGainTorqueController = 3
    ForceAxisImpedenceFeedbackController = 4
    PassThroughFeedbackController = 5

class TerminationHandlerType:
    NoopTerminationHandler = 1
    FinalPoseTerminationHandler = 2
    FinalJointTerminationHandler = 3
    LinearTrajectoryGeneratorWithTimeAndGoalTerminationHandler = 4
    ContactTerminationHandler = 5
    TimeTerminationHandler = 6

class SkillStatus: 
    TO_START = 0
    RUNNING = 1
    FINISHED = 2 