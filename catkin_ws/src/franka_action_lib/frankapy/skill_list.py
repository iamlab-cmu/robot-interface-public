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
    def __init__(self, skill_desc=''):
        if len(skill_desc) == 0:
            skill_desc = GripperSkill.__name__
        super(GripperSkill, self).__init__(
              SkillType.GripperSkill,
              skill_desc,
              MetaSkillType.BaseMetaSkill,
              0,
              ['/franka_robot/camera'],
              TrajectoryGeneratorType.GripperTrajectoryGenerator,
              FeedbackControllerType.NoopFeedbackController,
              TerminationHandlerType.NoopTerminationHandler,
              1)

class JointDMPSkillWithJointPositionControl(BaseSkill):
    def __init__(self, skill_desc=''):
        if len(skill_desc) == 0:
            skill_desc = JointDMPSkillWithJointPositionControl.__name__
        super(JointDMPSkillWithJointPositionControl, self).__init__(
              SkillType.JointPositionSkill,
              skill_desc,
              MetaSkillType.BaseMetaSkill,
              0,
              ['/franka_robot/camera'],
              TrajectoryGeneratorType.JointDmpTrajectoryGenerator,
              FeedbackControllerType.NoopFeedbackController,
              TerminationHandlerType.TimeTerminationHandler,
              1)

class JointDMPSkillWithImpedanceControl(BaseSkill):
    def __init__(self, skill_desc=''):
        if len(skill_desc) == 0:
            skill_desc = \
                    JointDMPSkillWithImpedanceControl.__name__
        super(JointDMPSkillWithImpedanceControl, self).__init__(
              SkillType.ImpedanceControlSkill,
              skill_desc,
              MetaSkillType.BaseMetaSkill,
              0,
              ['/franka_robot/camera'],
              TrajectoryGeneratorType.JointDmpTrajectoryGenerator,
              FeedbackControllerType.JointImpedanceFeedbackController,
              TerminationHandlerType.TimeTerminationHandler,
              1)

class MinJerkJointPositionSkill(BaseSkill):
    def __init__(self, skill_desc=''):
        if len(skill_desc) == 0:
            skill_desc = MinJerkJointPositionSkill.__name__
        super(MinJerkJointPositionSkill, self).__init__(
              SkillType.JointPositionSkill,
              skill_desc,
              MetaSkillType.BaseMetaSkill,
              0,
              ['/franka_robot/camera'],
              TrajectoryGeneratorType.MinJerkJointTrajectoryGenerator,
              FeedbackControllerType.NoopFeedbackController,
              TerminationHandlerType.FinalJointTerminationHandler,
              1)

class MinJerkJointPositionSkillWithInternalJointImpedances(BaseSkill):
    def __init__(self, skill_desc=''):
        if len(skill_desc) == 0:
            skill_desc = MinJerkJointPositionSkillWithInternalJointImpedances.__name__
        super(MinJerkJointPositionSkillWithInternalJointImpedances, self).__init__(
              SkillType.JointPositionSkill,
              skill_desc,
              MetaSkillType.BaseMetaSkill,
              0,
              ['/franka_robot/camera'],
              TrajectoryGeneratorType.MinJerkJointTrajectoryGenerator,
              FeedbackControllerType.SetInternalImpedanceFeedbackController,
              TerminationHandlerType.FinalJointTerminationHandler,
              1)

class MinJerkCartesianPoseSkillWithImpedanceControl(BaseSkill):
    def __init__(self, skill_desc=''):
        if len(skill_desc) == 0:
            skill_desc = MinJerkCartesianPoseSkillWithImpedanceControl.__name__
        super(MinJerkCartesianPoseSkillWithImpedanceControl, self).__init__(
              SkillType.ImpedanceControlSkill,
              skill_desc,
              MetaSkillType.BaseMetaSkill,
              0,
              ['/franka_robot/camera'],
              TrajectoryGeneratorType.MinJerkPoseTrajectoryGenerator,
              FeedbackControllerType.CartesianImpedanceFeedbackController,
              TerminationHandlerType.FinalPoseTerminationHandler,
              1)

    def add_buffer_time_for_termination(self, buffer_time):
        self.add_termination_params([buffer_time])

class MinJerkCartesianPoseSkillWithCartesianPoseControl(BaseSkill):
    def __init__(self, skill_desc=''):
        if len(skill_desc) == 0:
            skill_desc = MinJerkCartesianPoseSkillWithCartesianPoseControl.__name__
        super(MinJerkCartesianPoseSkillWithCartesianPoseControl, self).__init__(
              SkillType.CartesianPoseSkill,
              skill_desc,
              MetaSkillType.BaseMetaSkill,
              0,
              ['/franka_robot/camera'],
              TrajectoryGeneratorType.MinJerkPoseTrajectoryGenerator,
              FeedbackControllerType.SetInternalImpedanceFeedbackController,
              TerminationHandlerType.FinalPoseTerminationHandler,
              1)

    def add_buffer_time_for_termination(self, buffer_time):
        self.add_termination_params([buffer_time])

class RelativeMinJerkCartesianPoseSkillWithImpedanceControl(BaseSkill):
    def __init__(self, skill_desc=''):
        if len(skill_desc) == 0:
            skill_desc = RelativeMinJerkCartesianPoseSkillWithImpedanceControl.__name__
        super(RelativeMinJerkCartesianPoseSkillWithImpedanceControl, self).__init__(
              SkillType.ImpedanceControlSkill,
              skill_desc,
              MetaSkillType.BaseMetaSkill,
              0,
              ['/franka_robot/camera'],
              TrajectoryGeneratorType.RelativeMinJerkPoseTrajectoryGenerator,
              FeedbackControllerType.CartesianImpedanceFeedbackController,
              TerminationHandlerType.FinalPoseTerminationHandler,
              1)

    def add_relative_motion_with_quaternion(self, time, position, quaternion):
        assert len(position) == 3, "Incorrect position to move to."
        assert len(quaternion) == 4, "Incorrect quaternion representation."
        self.add_trajectory_params([time] + position + quaternion)

class RelativeMinJerkCartesianPoseSkillWithImpedanceControlToContact(BaseSkill):
    def __init__(self, skill_desc=''):
        if len(skill_desc) == 0:
            skill_desc = \
                    RelativeMinJerkCartesianPoseSkillWithImpedanceControlToContact.__name__
        super(RelativeMinJerkCartesianPoseSkillWithImpedanceControlToContact, self).__init__(
              SkillType.ImpedanceControlSkill,
              skill_desc,
              MetaSkillType.BaseMetaSkill,
              0,
              ['/franka_robot/camera'],
              TrajectoryGeneratorType.RelativeMinJerkPoseTrajectoryGenerator,
              FeedbackControllerType.CartesianImpedanceFeedbackController,
              TerminationHandlerType.ContactTerminationHandler,
              1)

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
                RelativeMinJerkCartesianPoseSkillWithImpedanceControlToContact.get_default_torque_thresholds()
        force_thresholds = \
                RelativeMinJerkCartesianPoseSkillWithImpedanceControlToContact.get_default_force_thresholds()
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

class RelativeMinJerkCartesianPoseSkillWithCartesianPoseControl(BaseSkill):
    def __init__(self, skill_desc=''):
        if len(skill_desc) == 0:
            skill_desc = RelativeMinJerkCartesianPoseSkillWithCartesianPoseControl.__name__
        super(RelativeMinJerkCartesianPoseSkillWithCartesianPoseControl, self).__init__(
              SkillType.CartesianPoseSkill,
              skill_desc,
              MetaSkillType.BaseMetaSkill,
              0,
              ['/franka_robot/camera'],
              # Default options, we can later maybe provide options for user
              # to change these.
              TrajectoryGeneratorType.RelativeMinJerkPoseTrajectoryGenerator,
              FeedbackControllerType.SetInternalImpedanceFeedbackController,
              TerminationHandlerType.FinalPoseTerminationHandler,
              1)

    def add_relative_motion_with_quaternion(self, time, position, quaternion):
        assert len(position) == 3, "Incorrect position to move to."
        assert len(quaternion) == 4, "Incorrect quaternion representation."
        self.add_trajectory_params([time] + position + quaternion)

class RelativeMinJerkCartesianPoseSkillWithCartesianPoseControlToContact(BaseSkill):
    def __init__(self, skill_desc=''):
        if len(skill_desc) == 0:
            skill_desc = \
                    RelativeMinJerkCartesianPoseSkillWithCartesianPoseControlToContact.__name__
        super(RelativeMinJerkCartesianPoseSkillWithCartesianPoseControlToContact, self).__init__(
              SkillType.CartesianPoseSkill,
              skill_desc,
              MetaSkillType.BaseMetaSkill,
              0,
              ['/franka_robot/camera'],
              TrajectoryGeneratorType.RelativeMinJerkPoseTrajectoryGenerator,
              FeedbackControllerType.SetInternalImpedanceFeedbackController,
              TerminationHandlerType.ContactTerminationHandler,
              1)

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
                RelativeMinJerkCartesianPoseSkillWithCartesianPoseControlToContact.get_default_torque_thresholds()
        force_thresholds = \
                RelativeMinJerkCartesianPoseSkillWithCartesianPoseControlToContact.get_default_force_thresholds()
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

class MinJerkCartesianPoseSkillWithImpedanceControlToContact(BaseSkill):
    def __init__(self, skill_desc=''):
        if len(skill_desc) == 0:
            skill_desc = \
                    MinJerkCartesianPoseSkillWithImpedanceControlToContact.__name__
        super(MinJerkCartesianPoseSkillWithImpedanceControlToContact, self).__init__(
              SkillType.ImpedanceControlSkill,
              skill_desc,
              MetaSkillType.BaseMetaSkill,
              0,
              ['/franka_robot/camera'],
              TrajectoryGeneratorType.MinJerkPoseTrajectoryGenerator,
              FeedbackControllerType.CartesianImpedanceFeedbackController,
              TerminationHandlerType.ContactTerminationHandler,
              1)

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

    def add_buffer_time_for_termination(self, buffer_time):
        self.add_termination_params([buffer_time])

    def add_contact_termination_params(self, 
            buffer_time,
            lower_force_thresholds_accel,
            lower_force_thresholds_nominal):
        torque_thresholds = \
                MinJerkCartesianPoseSkillWithImpedanceControlToContact.get_default_torque_thresholds()
        force_thresholds = \
                MinJerkCartesianPoseSkillWithImpedanceControlToContact.get_default_force_thresholds()
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

class MinJerkCartesianPoseSkillWithCartesianPoseControlToContact(BaseSkill):
    def __init__(self, skill_desc=''):
        if len(skill_desc) == 0:
            skill_desc = \
                    MinJerkCartesianPoseSkillWithCartesianPoseControlToContact.__name__
        super(MinJerkCartesianPoseSkillWithCartesianPoseControlToContact, self).__init__(
              SkillType.CartesianPoseSkill,
              skill_desc,
              MetaSkillType.BaseMetaSkill,
              0,
              ['/franka_robot/camera'],
              TrajectoryGeneratorType.MinJerkPoseTrajectoryGenerator,
              FeedbackControllerType.NoopFeedbackController,
              TerminationHandlerType.ContactTerminationHandler,
              1)

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

    def add_buffer_time_for_termination(self, buffer_time):
        self.add_termination_params([buffer_time])

    def add_contact_termination_params(self, 
            buffer_time,
            lower_force_thresholds_accel,
            lower_force_thresholds_nominal):
        torque_thresholds = \
                MinJerkCartesianPoseSkillWithCartesianPoseControlToContact.get_default_torque_thresholds()
        force_thresholds = \
                MinJerkCartesianPoseSkillWithCartesianPoseControlToContact.get_default_force_thresholds()
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

class StayInInitialPoseSkillWithCartesianImpedance(BaseSkill):
    def __init__(self, skill_desc=''):
        if len(skill_desc) == 0:
            skill_desc = StayInInitialPoseSkillWithCartesianImpedance.__name__
        super(StayInInitialPoseSkillWithCartesianImpedance, self).__init__(
              SkillType.ImpedanceControlSkill,
              skill_desc,
              MetaSkillType.BaseMetaSkill,
              0,
              ['/franka_robot/camera'],
              TrajectoryGeneratorType.StayInInitialPoseTrajectoryGenerator,
              FeedbackControllerType.CartesianImpedanceFeedbackController,
              TerminationHandlerType.TimeTerminationHandler,
              1)

class StayInInitialPoseSkillWithJointImpedance(BaseSkill):
    def __init__(self, skill_desc=''):
        if len(skill_desc) == 0:
            skill_desc = StayInInitialPoseSkillWithJointImpedance.__name__
        super(StayInInitialPoseSkillWithJointImpedance, self).__init__(
              SkillType.ImpedanceControlSkill,
              skill_desc,
              MetaSkillType.BaseMetaSkill,
              0,
              ['/franka_robot/camera'],
              TrajectoryGeneratorType.StayInInitialPoseTrajectoryGenerator,
              FeedbackControllerType.JointImpedanceFeedbackController,
              TerminationHandlerType.TimeTerminationHandler,
              1)

class ForceTorqueSkill(BaseSkill):
    def __init__(self, skill_desc=''):
        if len(skill_desc) == 0:
            skill_desc = ForceTorqueSkill.__name__
        super(ForceTorqueSkill, self).__init__(
              SkillType.ForceTorqueSkill,
              skill_desc,
              MetaSkillType.BaseMetaSkill,
              0,
              ['/franka_robot/camera'],
              TrajectoryGeneratorType.ImpulseTrajectoryGenerator,
              FeedbackControllerType.PassThroughFeedbackController,
              TerminationHandlerType.TimeTerminationHandler,
              1)

class ForceAlongAxisSkill(BaseSkill):
    def __init__(self, skill_desc=''):
        if len(skill_desc) == 0:
            skill_desc = ForceTorqueSkill.__name__
        super(ForceAlongAxisSkill, self).__init__(
              SkillType.ForceTorqueSkill,
              skill_desc,
              MetaSkillType.BaseMetaSkill,
              0,
              ['/franka_robot/camera'],
              TrajectoryGeneratorType.ImpulseTrajectoryGenerator,
              FeedbackControllerType.ForceAxisImpedenceFeedbackController,
              TerminationHandlerType.TimeTerminationHandler,
              1)
