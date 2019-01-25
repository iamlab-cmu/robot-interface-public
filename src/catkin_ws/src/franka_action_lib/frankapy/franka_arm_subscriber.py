import sys, logging
from multiprocessing import Queue
from Queue import Empty
from time import sleep

import numpy as np
import rospy
from franka_action_lib.msg import RobotState

from franka_constants import FrankaConstants as FC
from utils import franka_pose_to_rigid_transform

class FrankaArmSubscriber:

    def __init__(self, new_ros_node=True, topic_name='/robot_state_publisher_node/robot_state'):
        self._data_q = Queue(maxsize=1)

        def callback(data):
            if self._data_q.full():
                try:
                    self._data_q.get_nowait()
                except Empty:
                    pass
            self._data_q.put(data)

        if new_ros_node:
            rospy.init_node('FrankaArmSubscriber', anonymous=True)

        rospy.Subscriber(topic_name, RobotState, callback)

    def get_data(self):
        '''Get all fields of current robot data in a dict.

        Returns:
            out: dict of robot state
        '''
        ros_data = self._data_q.get(block=True)

        data = {
            'pose_desired': franka_pose_to_rigid_transform(ros_data.pose_desired),
            'pose': franka_pose_to_rigid_transform(ros_data.pose),
            'joint_torques': np.array(ros_data.joint_torques),
            'joint_torques_derivative': np.array(ros_data.joint_torques_derivative),
            'joints': np.array(ros_data.joints),
            'joints_desired': np.array(ros_data.joints_desired),
            'joint_velocities': np.array(ros_data.joint_velocities),
            'time_since_skill_started': ros_data.time_since_skill_started            
        }

        return data

    def get_pose(self):
        '''Get the current pose.

        Returns:
            out : RigidTransform
        '''
        return self.get_data()['pose']

    def get_joints(self):
        '''Get the current joint configuration.

        Returns:
            out : ndarray of shape (7,)
        '''
        return self.get_data()['joints']

    def get_joint_torques(self):
        '''Get the current joint torques.

        Returns:
            out : ndarray of shape (7,)
        '''
        return self.get_data()['joint_torques']

    def get_joint_velocities(self):
        '''Get the current joint velocities.

        Returns:
            out : ndarray of shape (7,)
        '''
        return self.get_data()['joint_velocities']