#!/usr/bin/env python

import roslib
roslib.load_manifest('franka_action_lib')
import rospy
import actionlib
import numpy as np

from franka_action_lib.msg import ExecuteSkillAction, ExecuteSkillGoal

class BaseSkill(object):
    def __init__(self, 
                 skill_type,
                 sensor_topics,
                 trajectory_generator_type,
                 feedback_controller_type,
                 termination_type,
                 timer_type):
        self._skill_type = skill_type
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


    def add_initial_sensor_values(self, values):
        self._initial_sensor_values = values
        self._sensor_value_sizes = [len(values)]

    def add_trajectory_params(self, params):
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
        print(feedback)

class GripperWithDefaultSensorSkill(BaseSkill):
    def __init__(self, 
                skill_type=1,
                trajectory_generator_type=5,
                feedback_controller_type=1,
                termination_type=1,
                timer_type=1):
        super(GripperWithDefaultSensorSkill, self).__init__(
              skill_type,
              ['/franka_robot/camera'],
              trajectory_generator_type,
              feedback_controller_type,
              termination_type,
              timer_type)

class ArmMoveToGoalWithDefaultSensorSkill(BaseSkill):
    def __init__(self, 
                skill_type=0,
                trajectory_generator_type=4,
                feedback_controller_type=2,
                termination_type=4,
                timer_type=1):
        super(ArmMoveToGoalWithDefaultSensorSkill, self).__init__(
              skill_type,
              ['/franka_robot/camera'],
              trajectory_generator_type,
              feedback_controller_type,
              termination_type,
              timer_type)

class ArmMoveToGoalContactWithDefaultSensorSkill(BaseSkill):
    def __init__(self, 
                skill_type=0,
                trajectory_generator_type=4,
                feedback_controller_type=2,
                termination_type=5,
                timer_type=1):
        super(ArmMoveToGoalContactWithDefaultSensorSkill, self).__init__(
              skill_type,
              ['/franka_robot/camera'],
              trajectory_generator_type,
              feedback_controller_type,
              termination_type,
              timer_type)

class NoOpSkill(BaseSkill):
    def __init__(self, 
                skill_type=0,
                trajectory_generator_type=1,
                feedback_controller_type=1,
                termination_type=1,
                timer_type=1):
        super(NoOpSkill, self).__init__(
              skill_type,
              ['/franka_robot/camera'],
              trajectory_generator_type,
              feedback_controller_type,
              termination_type,
              timer_type)

class StayInPositionWithDefaultSensorSkill(BaseSkill):
    def __init__(self, 
                skill_type=0,
                trajectory_generator_type=6,
                feedback_controller_type=2,
                termination_type=6,
                timer_type=1):
        super(StayInPositionWithDefaultSensorSkill, self).__init__(
              skill_type,
              ['/franka_robot/camera'],
              trajectory_generator_type,
              feedback_controller_type,
              termination_type,
              timer_type)