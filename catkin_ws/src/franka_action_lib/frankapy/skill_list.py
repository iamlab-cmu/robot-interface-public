#!/usr/bin/env python

import roslib
roslib.load_manifest('franka_action_lib')
import rospy
import actionlib
import numpy as np
from .iam_robolib_common_definitions import *

from franka_action_lib.msg import ExecuteSkillAction, ExecuteSkillGoal

class BaseSkill(object):
    def __init__(self, 
                 skill_type,
                 skill_desc,
                 meta_skill_type,
                 meta_skill_id,
                 sensor_topics,
                 trajectory_generator_type,
                 feedback_controller_type,
                 termination_type,
                 timer_type):
        self._skill_type = skill_type
        self._skill_desc = skill_desc
        self._meta_skill_type = meta_skill_type
        self._meta_skill_id = meta_skill_id
        self._sensor_topics = sensor_topics
        self._trajectory_generator_type = trajectory_generator_type
        self._feedback_controller_type = feedback_controller_type
        self._termination_type = termination_type
        self._timer_type = timer_type

        self._sensor_value_sizes = 0
        self._initial_sensor_values = []

        # Add trajectory params
        self._trajectory_generator_params = []
        self._num_trajectory_generator_params = 0

        # Add feedback controller params
        self._feedback_controller_params = []
        self._num_feedback_controller_params = 0

        # Add termination params
        self._termination_params = []
        self._num_termination_params = 0

        # Add timer params
        self._timer_params = []
        self._num_timer_params = 0

    def set_meta_skill_type(self, meta_skill_type):
        self._meta_skill_type = meta_skill_type

    def set_meta_skill_id(self, meta_skill_id):
        self._meta_skill_id = meta_skill_id

    def add_initial_sensor_values(self, values):
        self._initial_sensor_values = values
        self._sensor_value_sizes = [len(values)]

    def add_trajectory_params(self, params):
        assert type(params) is list, \
                "Invalid type of params provided {}".format(params)
        self._trajectory_generator_params = params
        self._num_trajectory_generator_params = len(params)

    def add_feedback_controller_params(self, params):
        self._feedback_controller_params = params
        self._num_feedback_controller_params = len(params)

    def add_termination_params(self, params):
        self._termination_params = params
        self._num_termination_params = len(params)

    def add_timer_params(self, params):
        self._timer_params = params
        self._num_timer_params = len(params)

    # Add checks for these
    def create_goal(self):
        goal = ExecuteSkillGoal()
        goal.skill_type = self._skill_type
        goal.skill_description = self._skill_desc
        goal.meta_skill_type = self._meta_skill_type
        goal.meta_skill_id = self._meta_skill_id
        goal.sensor_topics = self._sensor_topics
        goal.initial_sensor_values = self._initial_sensor_values
        goal.sensor_value_sizes = self._sensor_value_sizes
        goal.traj_gen_type = self._trajectory_generator_type
        goal.traj_gen_params = self._trajectory_generator_params
        goal.num_traj_gen_params = self._num_trajectory_generator_params
        goal.feedback_controller_type = self._feedback_controller_type
        goal.feedback_controller_params = self._feedback_controller_params
        goal.num_feedback_controller_params = \
                self._num_feedback_controller_params
        goal.termination_type = self._termination_type
        goal.termination_params = self._termination_params
        goal.num_termination_params = self._num_termination_params
        goal.timer_type = self._timer_type
        goal.timer_params = self._timer_params
        goal.num_timer_params = self._num_timer_params
        return goal

    def feedback_callback(self, feedback):
        pass

class GripperSkill(BaseSkill):
    def __init__(self, 
                skill_type=SkillType.GripperSkill,
                skill_desc='',
                meta_skill_type=MetaSkillType.BaseMetaSkill,
                meta_skill_id=0,
                trajectory_generator_type=TrajectoryGeneratorType.GripperTrajectoryGenerator,
                feedback_controller_type=FeedbackControllerType.NoopFeedbackController,
                termination_type=TerminationHandlerType.NoopTerminationHandler,
                timer_type=1):
        if len(skill_desc) == 0:
            skill_desc = GripperSkill.__name__
        super(GripperSkill, self).__init__(
              skill_type,
              skill_desc,
              meta_skill_type,
              meta_skill_id,
              ['/franka_robot/camera'],
              trajectory_generator_type,
              feedback_controller_type,
              termination_type,
              timer_type)

class JointPoseDMPSkill(BaseSkill):
    def __init__(self, 
                skill_type=SkillType.JointPositionSkill,
                skill_desc='',
                meta_skill_type=MetaSkillType.BaseMetaSkill,
                meta_skill_id=0,
                trajectory_generator_type=TrajectoryGeneratorType.JointDmpTrajectoryGenerator,
                feedback_controller_type=FeedbackControllerType.NoopFeedbackController,
                termination_type=TerminationHandlerType.TimeTerminationHandler,
                timer_type=1):
        if len(skill_desc) == 0:
            skill_desc = JointPoseSkill.__name__
        super(JointPoseDMPSkill, self).__init__(
              skill_type,
              skill_desc,
              meta_skill_type,
              meta_skill_id,
              ['/franka_robot/camera'],
              trajectory_generator_type,
              feedback_controller_type,
              termination_type,
              timer_type)

class JointPoseDMPWithTorqueControlSkill(BaseSkill):
    def __init__(self, 
                skill_type=SkillType.JointPositionSkill,
                skill_desc='',
                meta_skill_type=MetaSkillType.BaseMetaSkill,
                meta_skill_id=0,
                trajectory_generator_type=TrajectoryGeneratorType.JointDmpTrajectoryGenerator,
                feedback_controller_type=FeedbackControllerType.JointImpedanceFeedbackController,
                termination_type=TerminationHandlerType.TimeTerminationHandler,
                timer_type=1):
        if len(skill_desc) == 0:
            skill_desc = \
                    JointPoseDMPWithTorqueControlSkill.__name__
        super(JointPoseDMPWithTorqueControlSkill, self).__init__(
              skill_type,
              skill_desc,
              meta_skill_type,
              meta_skill_id,
              ['/franka_robot/camera'],
              trajectory_generator_type,
              feedback_controller_type,
              termination_type,
              timer_type)

class JointPoseSkill(BaseSkill):
    def __init__(self, 
                skill_type=SkillType.JointPositionSkill,
                skill_desc='',
                meta_skill_type=MetaSkillType.BaseMetaSkill,
                meta_skill_id=0,
                trajectory_generator_type=TrajectoryGeneratorType.LinearJointTrajectoryGenerator,
                feedback_controller_type=FeedbackControllerType.NoopFeedbackController,
                termination_type=TerminationHandlerType.FinalJointTerminationHandler,
                timer_type=1):
        if len(skill_desc) == 0:
            skill_desc = JointPoseSkill.__name__
        super(JointPoseSkill, self).__init__(
              skill_type,
              skill_desc,
              meta_skill_type,
              meta_skill_id,
              ['/franka_robot/camera'],
              trajectory_generator_type,
              feedback_controller_type,
              termination_type,
              timer_type)

class JointPoseMinJerkSkill(BaseSkill):
    def __init__(self, 
                skill_type=SkillType.JointPositionSkill,
                skill_desc='',
                meta_skill_type=MetaSkillType.BaseMetaSkill,
                meta_skill_id=0,
                trajectory_generator_type=TrajectoryGeneratorType.MinJerkJointTrajectoryGenerator,
                feedback_controller_type=FeedbackControllerType.NoopFeedbackController,
                termination_type=TerminationHandlerType.FinalJointTerminationHandler,
                timer_type=1):
        if len(skill_desc) == 0:
            skill_desc = JointPoseMinJerkSkill.__name__
        super(JointPoseMinJerkSkill, self).__init__(
              skill_type,
              skill_desc,
              meta_skill_type,
              meta_skill_id,
              ['/franka_robot/camera'],
              trajectory_generator_type,
              feedback_controller_type,
              termination_type,
              timer_type)

class JointPoseMinJerkWithJointImpedancesSkill(BaseSkill):
    def __init__(self, 
                skill_type=SkillType.JointPositionSkill,
                skill_desc='',
                meta_skill_type=MetaSkillType.BaseMetaSkill,
                meta_skill_id=0,
                trajectory_generator_type=TrajectoryGeneratorType.MinJerkJointTrajectoryGenerator,
                feedback_controller_type=FeedbackControllerType.SetInternalImpedanceFeedbackController,
                termination_type=TerminationHandlerType.FinalJointTerminationHandler,
                timer_type=1):
        if len(skill_desc) == 0:
            skill_desc = JointPoseMinJerkWithJointImpedancesSkill.__name__
        super(JointPoseMinJerkWithJointImpedancesSkill, self).__init__(
              skill_type,
              skill_desc,
              meta_skill_type,
              meta_skill_id,
              ['/franka_robot/camera'],
              trajectory_generator_type,
              feedback_controller_type,
              termination_type,
              timer_type)

class ArmMoveToGoalSkill(BaseSkill):
    def __init__(self, 
                skill_type=SkillType.ImpedanceControlSkill,
                skill_desc='',
                meta_skill_type=MetaSkillType.BaseMetaSkill,
                meta_skill_id=0,
                trajectory_generator_type=TrajectoryGeneratorType.LinearPoseTrajectoryGenerator,
                feedback_controller_type=FeedbackControllerType.CartesianImpedanceFeedbackController,
                termination_type=TerminationHandlerType.FinalPoseTerminationHandler,
                timer_type=1):
        if len(skill_desc) == 0:
            skill_desc = ArmMoveToGoalSkill.__name__
        super(ArmMoveToGoalSkill, self).__init__(
              skill_type,
              skill_desc,
              meta_skill_type,
              meta_skill_id,
              ['/franka_robot/camera'],
              trajectory_generator_type,
              feedback_controller_type,
              termination_type,
              timer_type)

    def add_buffer_time_for_termination(self, buffer_time):
        self.add_termination_params([buffer_time])

class ArmRelativeMotionSkill(BaseSkill):
    def __init__(self, 
                skill_type=SkillType.ImpedanceControlSkill,
                skill_desc='',
                meta_skill_type=MetaSkillType.BaseMetaSkill,
                meta_skill_id=0,
                trajectory_generator_type=TrajectoryGeneratorType.RelativeLinearPoseTrajectoryGenerator,
                feedback_controller_type=FeedbackControllerType.CartesianImpedanceFeedbackController,
                termination_type=TerminationHandlerType.FinalPoseTerminationHandler,
                timer_type=1):
        if len(skill_desc) == 0:
            skill_desc = ArmRelativeMotionSkill.__name__
        super(ArmRelativeMotionSkill, self).__init__(
              skill_type,
              skill_desc,
              meta_skill_type,
              meta_skill_id,
              ['/franka_robot/camera'],
              trajectory_generator_type,
              feedback_controller_type,
              termination_type,
              timer_type)

    def add_relative_motion_with_quaternion(self, time, position, quaternion):
        assert len(position) == 3, "Incorrect position to move to."
        assert len(quaternion) == 4, "Incorrect quaternion representation."
        self.add_trajectory_params([time] + position + quaternion)

class ArmRelativeMotionToContactSkill(BaseSkill):
    def __init__(self, 
                skill_type=SkillType.ImpedanceControlSkill,
                skill_desc='',
                meta_skill_type=MetaSkillType.BaseMetaSkill,
                meta_skill_id=0,
                trajectory_generator_type=TrajectoryGeneratorType.RelativeLinearPoseTrajectoryGenerator,
                feedback_controller_type=FeedbackControllerType.CartesianImpedanceFeedbackController,
                termination_type=TerminationHandlerType.ContactTerminationHandler,
                timer_type=1):
        if len(skill_desc) == 0:
            skill_desc = \
                    ArmRelativeMotionToContactSkill.__name__
        super(ArmRelativeMotionToContactSkill, self).__init__(
              skill_type,
              skill_desc,
              meta_skill_type,
              meta_skill_id,
              ['/franka_robot/camera'],
              trajectory_generator_type,
              feedback_controller_type,
              termination_type,
              timer_type)

    @staticmethod
    def get_default_torque_thresholds():
        lower_torque_thresholds_accel = \
                [20.0,20.0,18.0,18.0,16.0,14.0,12.0]
        upper_torque_thresholds_accel = \
                [120.0,120.0,120.0,118.0,116.0,114.0,112.0]
        lower_torque_thresholds_nominal = \
                [20.0,20.0,18.0,18.0,16.0,14.0,12.0]
        upper_torque_thresholds_nominal = \
                [120.0,120.0,118.0,118.0,116.0,114.0,112.0]

        return {
            'lower_torque_thresholds_accel': lower_torque_thresholds_accel,
            'upper_torque_thresholds_accel': upper_torque_thresholds_accel,
            'lower_torque_thresholds_nominal': lower_torque_thresholds_nominal,
            'upper_torque_thresholds_nominal': upper_torque_thresholds_nominal,
        }

    @staticmethod
    def get_default_force_thresholds():
        lower_force_thresholds_accel = \
                [10.0,10.0,10.0,10.0,10.0,10.0]
        upper_force_thresholds_accel = \
                [120.0,120.0,120.0,125.0,125.0,125.0]
        lower_force_thresholds_nominal = \
                [10.0,10.0,10.0,10.0,10.0,10.0]
        upper_force_thresholds_nominal = \
                [120.0,120.0,120.0,125.0,125.0,125.0]
        return {
            'lower_force_thresholds_accel': lower_force_thresholds_accel,
            'upper_force_thresholds_accel': upper_force_thresholds_accel,
            'lower_force_thresholds_nominal': lower_force_thresholds_nominal,
            'upper_force_thresholds_nominal': upper_force_thresholds_nominal
        }

    def add_controller_stiffness_params(self,
                                        translational_stiffness=600,
                                        rotational_stiffness=50):
        self.add_feedback_controller_params(
                [translational_stiffness, rotational_stiffness])

    def add_traj_params_with_quaternion(self, time, position, quaternion):
        '''Add trajectory parameters with desired orientation as quaternion

        time: time for the trajectory.
        position: Relative movement for the trajectory.
        quaternion: Desired orientation for the trajectory.
        '''
        assert len(position) == 3, "Incorrect position given"
        assert len(quaternion) == 4, "Incorrect quaternion representation"
        assert type(position) is list, "Incorrect position type"
        assert type(quaternion) is list, "Incorrect quaternion type"
        self.add_trajectory_params([time] + position + quaternion)

    def add_collision_termination_params(self, buffer_time,
            lower_torque_thresholds_accel=[10.0,10.0,10.0,10.0,10.0,10.0,10.0],
            upper_torque_thresholds_accel=[120.0,120.0,118.0,118.0,116.0,114.0,112.0],
            lower_torque_thresholds_nominal=[10.0,10.0,10.0,10.0,10.0,10.0,10.0],
            upper_torque_thresholds_nominal=[120.0,120.0,118.0,118.0,116.0,114.0,112.0],
            lower_force_thresholds_accel=[10.0,10.0,10.0,10.0,10.0,10.0],
            upper_force_thresholds_accel=[120.0,120.0,120.0,125.0,125.0,125.0],
            lower_force_thresholds_nominal=[10.0,10.0,10.0,10.0,10.0,10.0],
            upper_force_thresholds_nominal=[120.0,120.0,120.0,125.0,125.0,125.0]):
        collision_termination_params = lower_torque_thresholds_accel \
                + upper_torque_thresholds_accel \
                + lower_torque_thresholds_nominal \
                + upper_torque_thresholds_nominal \
                + lower_force_thresholds_accel \
                + upper_force_thresholds_accel \
                + lower_force_thresholds_nominal \
                + upper_force_thresholds_nominal

        self.add_termination_params([buffer_time] + collision_termination_params)
    
    def add_contact_termination_params(self, 
            buffer_time,
            lower_force_thresholds_accel,
            lower_force_thresholds_nominal):
        torque_thresholds = \
                ArmRelativeMotionToContactSkill.get_default_torque_thresholds()
        force_thresholds = \
                ArmRelativeMotionToContactSkill.get_default_force_thresholds()
        params = [buffer_time] \
                + torque_thresholds['lower_torque_thresholds_accel'] \
                + torque_thresholds['upper_torque_thresholds_accel'] \
                + torque_thresholds['lower_torque_thresholds_nominal'] \
                + torque_thresholds['upper_torque_thresholds_nominal'] \
                + lower_force_thresholds_accel \
                + force_thresholds['upper_force_thresholds_accel'] \
                + lower_force_thresholds_nominal \
                + force_thresholds['upper_force_thresholds_nominal']

        self.add_termination_params(params)

class ArmMoveToGoalContactSkill(BaseSkill):
    def __init__(self, 
                skill_type=SkillType.ImpedanceControlSkill,
                skill_desc='',
                meta_skill_type=MetaSkillType.BaseMetaSkill,
                meta_skill_id=0,
                trajectory_generator_type=TrajectoryGeneratorType.LinearPoseTrajectoryGenerator,
                feedback_controller_type=FeedbackControllerType.CartesianImpedanceFeedbackController,
                termination_type=TerminationHandlerType.ContactTerminationHandler,
                timer_type=1):
        if len(skill_desc) == 0:
            skill_desc = \
                    ArmMoveToGoalContactSkill.__name__
        super(ArmMoveToGoalContactSkill, self).__init__(
              skill_type,
              skill_desc,
              meta_skill_type,
              meta_skill_id,
              ['/franka_robot/camera'],
              trajectory_generator_type,
              feedback_controller_type,
              termination_type,
              timer_type)

    def add_buffer_time_for_termination(self, buffer_time):
        self.add_termination_params([buffer_time])

    def add_contact_termination_params(self, 
            buffer_time,
            lower_force_thresholds_accel,
            lower_force_thresholds_nominal):
        torque_thresholds = \
                ArmRelativeMotionToContactSkill.get_default_torque_thresholds()
        force_thresholds = \
                ArmRelativeMotionToContactSkill.get_default_force_thresholds()
        params = [buffer_time] \
                + torque_thresholds['lower_torque_thresholds_accel'] \
                + torque_thresholds['upper_torque_thresholds_accel'] \
                + torque_thresholds['lower_torque_thresholds_nominal'] \
                + torque_thresholds['upper_torque_thresholds_nominal'] \
                + lower_force_thresholds_accel \
                + force_thresholds['upper_force_thresholds_accel'] \
                + lower_force_thresholds_nominal \
                + force_thresholds['upper_force_thresholds_nominal']

        self.add_termination_params(params)

class StayInPositionSkill(BaseSkill):
    def __init__(self, 
                skill_type=SkillType.ImpedanceControlSkill,
                skill_desc='',
                meta_skill_type=MetaSkillType.BaseMetaSkill,
                meta_skill_id=0,
                trajectory_generator_type=TrajectoryGeneratorType.StayInInitialPositionTrajectoryGenerator,
                feedback_controller_type=FeedbackControllerType.CartesianImpedanceFeedbackController,
                termination_type=TerminationHandlerType.TimeTerminationHandler,
                timer_type=1):
        if len(skill_desc) == 0:
            skill_desc = StayInPositionSkill.__name__
        super(StayInPositionSkill, self).__init__(
              skill_type,
              skill_desc,
              meta_skill_type,
              meta_skill_id,
              ['/franka_robot/camera'],
              trajectory_generator_type,
              feedback_controller_type,
              termination_type,
              timer_type)

class StayInPositionWithSelectiveComplianceSkill(BaseSkill):
    def __init__(self, 
                skill_type=SkillType.ImpedanceControlSkill,
                skill_desc='',
                meta_skill_type=MetaSkillType.BaseMetaSkill,
                meta_skill_id=0,
                trajectory_generator_type=TrajectoryGeneratorType.StayInInitialPositionTrajectoryGenerator,
                feedback_controller_type=FeedbackControllerType.JointImpedanceFeedbackController,
                termination_type=TerminationHandlerType.TimeTerminationHandler,
                timer_type=1):
        if len(skill_desc) == 0:
            skill_desc = StayInPositionWithSelectiveComplianceSkill.__name__
        super(StayInPositionWithSelectiveComplianceSkill, self).__init__(
              skill_type,
              skill_desc,
              meta_skill_type,
              meta_skill_id,
              ['/franka_robot/camera'],
              trajectory_generator_type,
              feedback_controller_type,
              termination_type,
              timer_type)

class ForceTorqueSkill(BaseSkill):
    def __init__(self, 
                skill_type=SkillType.ForceTorqueSkill,
                skill_desc='',
                meta_skill_type=MetaSkillType.BaseMetaSkill,
                meta_skill_id=0,
                trajectory_generator_type=TrajectoryGeneratorType.ImpulseTrajectoryGenerator,
                feedback_controller_type=FeedbackControllerType.PassThroughFeedbackController,
                termination_type=TerminationHandlerType.TimeTerminationHandler,
                timer_type=1):
        if len(skill_desc) == 0:
            skill_desc = ForceTorqueSkill.__name__
        super(ForceTorqueSkill, self).__init__(
              skill_type,
              skill_desc,
              meta_skill_type,
              meta_skill_id,
              ['/franka_robot/camera'],
              trajectory_generator_type,
              feedback_controller_type,
              termination_type,
              timer_type)

class ForceAlongAxisSkill(BaseSkill):
    def __init__(self, 
                skill_type=SkillType.ForceTorqueSkill,
                skill_desc='',
                meta_skill_type=MetaSkillType.BaseMetaSkill,
                meta_skill_id=0,
                trajectory_generator_type=TrajectoryGeneratorType.ImpulseTrajectoryGenerator,
                feedback_controller_type=FeedbackControllerType.ForceAxisImpedenceFeedbackController,
                termination_type=TerminationHandlerType.TimeTerminationHandler,
                timer_type=1):
        if len(skill_desc) == 0:
            skill_desc = ForceTorqueSkill.__name__
        super(ForceAlongAxisSkill, self).__init__(
              skill_type,
              skill_desc,
              meta_skill_type,
              meta_skill_id,
              ['/franka_robot/camera'],
              trajectory_generator_type,
              feedback_controller_type,
              termination_type,
              timer_type)
