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
    CartesianPoseSkill = 0
    ForceTorqueSkill = 1
    GripperSkill = 2
    ImpedanceControlSkill = 3
    JointPositionSkill = 4

class MetaSkillType:
    BaseMetaSkill = 0
    JointPositionContinuousSkill = 1

class TrajectoryGeneratorType:
    GripperTrajectoryGenerator = 0
    ImpulseTrajectoryGenerator = 1
    JointDmpTrajectoryGenerator = 2
    LinearPoseTrajectoryGenerator = 3
    LinearJointTrajectoryGenerator = 4
    MinJerkJointTrajectoryGenerator = 5
    MinJerkPoseTrajectoryGenerator = 6
    PoseDmpTrajectoryGenerator = 7
    RelativeLinearPoseTrajectoryGenerator = 8
    RelativeMinJerkPoseTrajectoryGenerator = 9
    SineJointTrajectoryGenerator = 10
    SinePoseTrajectoryGenerator = 11
    StayInInitialJointsTrajectoryGenerator = 12
    StayInInitialPoseTrajectoryGenerator = 13

class FeedbackControllerType:
    CartesianImpedanceFeedbackController = 0
    ForceAxisImpedenceFeedbackController = 1
    JointImpedanceFeedbackController = 2
    NoopFeedbackController = 3
    PassThroughFeedbackController = 4
    SetInternalImpedanceFeedbackController = 5

class TerminationHandlerType:
    ContactTerminationHandler = 0
    FinalJointTerminationHandler = 1
    FinalPoseTerminationHandler = 2
    NoopTerminationHandler = 3
    TimeTerminationHandler = 4

class SkillStatus: 
    TO_START = 0
    RUNNING = 1
    FINISHED = 2 