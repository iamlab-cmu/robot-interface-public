import logging
import math

class FrankaConstants:

    LOGGING_LEVEL = logging.INFO

    EMPTY_SENSOR_VALUES = [0]

    # translational stiffness, rotational stiffness
    DEFAULT_TORQUE_CONTROLLER_PARAMS = [600, 50]

    # buffer time
    DEFAULT_TERM_PARAMS = [1]

    HOME_JOINTS = [0, -math.pi/4, 0, -3 * math.pi/4, 0, math.pi/2, math.pi/4]
