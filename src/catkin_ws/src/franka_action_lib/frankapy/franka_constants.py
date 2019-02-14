import logging
import math
import numpy as np
from autolab_core import RigidTransform

class FrankaConstants:
    '''
    All units are SI. 
    '''

    LOGGING_LEVEL = logging.INFO

    EMPTY_SENSOR_VALUES = [0]

    # translational stiffness, rotational stiffness
    DEFAULT_TORQUE_CONTROLLER_PARAMS = [600, 50]
    DEFAULT_FORCE_AXIS_CONTROLLER_PARAMS = [600, 0]

    # buffer time
    DEFAULT_TERM_PARAMS = [1]

    HOME_JOINTS = [0, -math.pi / 4, 0, -3 * math.pi / 4, 0, math.pi / 2, math.pi / 4]
    HOME_POSE = RigidTransform(rotation=np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1],
        ]), translation=np.array([0.3069, 0, 0.4867]),
        from_frame='franka_tool', to_frame='world')
    READY_JOINTS = [0, -math.pi/4, 0, -2.85496998, 0, 2.09382820,  math.pi/4]
    READY_POSE = RigidTransform(rotation=np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1],
        ]), translation=np.array([0.3069, 0, 0.2867]),
        from_frame='franka_tool', to_frame='world')

    # See https://frankaemika.github.io/docs/control_parameters.html
    JOINT_LIMITS_MIN = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
    JOINT_LIMITS_MAX = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]

    GRIPPER_WIDTH_MAX = 0.08
    GRIPPER_WIDTH_MIN = 0
    GRIPPER_WAIT_TIME = 0
    GRIPPER_MAX_FORCE = 80

    MAX_LIN_MOMENTUM = 20
    MAX_ANG_MOMENTUM = 2
    MAX_LIN_MOMENTUM_CONSTRAINED = 100

    ROS_ROBOLIB_STATUS_PUBLISHER_NAME = '/robolib_status_publisher_node/robolib_status'
    ROS_EXECUTE_SKILL_ACTION_SERVER_NAME = '/execute_skill_action_server_node/execute_skill'

    DEFAULT_ROBOLIB_TIMEOUT = 5 
    ACTION_WAIT_LOOP_TIME = 0.001

    GRIPPER_CMD_SLEEP_TIME = 0.2