import sys, signal
from time import time, sleep
from multiprocessing import Queue
try:
    from Queue import Empty
except:
    from queue import Empty
import numpy as np
from autolab_core import RigidTransform

import roslib
roslib.load_manifest('franka_action_lib')
import rospy
import actionlib
from franka_action_lib.msg import ExecuteSkillAction, RobolibStatus

from .skill_list import *
from .exceptions import *
from .franka_arm_subscriber import FrankaArmSubscriber
from .franka_constants import FrankaConstants as FC

class FrankaArm:

    def __init__(self, rosnode_name='franka_arm_client'):
        self._connected = False 
        self._in_skill = False

        # set signal handler to handle ctrl+c and kill sigs
        signal.signal(signal.SIGINT, self._sigint_handler_gen())

        # init ROS
        rospy.init_node(rosnode_name, disable_signals=True)

        self._robolib_status_q = Queue(maxsize=1)
        def robolib_status_callback(robolib_status):
            if self._robolib_status_q.full():
                try:
                    self._robolib_status_q.get_nowait()
                except Empty:
                    pass
            self._robolib_status_q.put(robolib_status)
        rospy.Subscriber(FC.ROS_ROBOLIB_STATUS_PUBLISHER_NAME, RobolibStatus, robolib_status_callback)

        self._sub = FrankaArmSubscriber(new_ros_node=False)
        self._client = actionlib.SimpleActionClient(FC.ROS_EXECUTE_SKILL_ACTION_SERVER_NAME, ExecuteSkillAction)
        self._client.wait_for_server()
        self.wait_for_robolib()
        
        # done init ROS
        self._connected = True

        # set default identity tool delta pose
        self._tool_delta_pose = RigidTransform(from_frame='franka_tool', to_frame='franka_tool_base')

    def wait_for_robolib(self, timeout=None):
        '''Blocks execution until robolib gives ready signal.    
        '''
        timeout = FC.DEFAULT_ROBOLIB_TIMEOUT if timeout is None else timeout
        try:
            robolib_status = self._robolib_status_q.get(block=True, timeout=timeout)
        except Empty:
            raise FrankaArmCommException('could not get status after {}s'.format(timeout))
        if not robolib_status.is_ready:
            raise FrankaArmRobolibNotReadyException()

    def _sigint_handler_gen(self):
        def sigint_handler(sig, frame):
            if self._connected and self._in_skill:
                self._client.cancel_goal()
            sys.exit(0)

        return sigint_handler

    def _send_goal(self, goal, cb):
        '''
        Raises:
            FrankaArmCommException if a timeout is reached
            FrankaArmException if robolib gives an error
            FrankaArmRobolibNotReadyException if robolib is not ready
        '''
        self._client.send_goal(goal, feedback_cb=cb)
        self._in_skill = True

        done = False
        while not done:
            try:
                robolib_status = self._robolib_status_q.get(block=True, timeout=FC.DEFAULT_ROBOLIB_TIMEOUT)
            except Empty:
                raise FrankaArmCommException('timeout reached after {}s'.format(FC.DEFAULT_ROBOLIB_TIMEOUT))

            if rospy.is_shutdown():
                raise RuntimeError('rospy is down!')
            elif robolib_status.error_description:
                raise FrankaArmException(robolib_status.error_description)
            elif not robolib_status.is_ready:
                raise FrankaArmRobolibNotReadyException()
            done = self._client.wait_for_result(rospy.Duration.from_sec(FC.ACTION_WAIT_LOOP_TIME))            

        self._in_skill = False
        return self._client.get_result()

    '''
    Controls
    '''

    def goto_pose(self, tool_pose, duration=3):
        '''Commands Arm to the given pose via linear interpolation

        Args:
            tool_pose (RigidTransform) : End-effector pose in tool frame
            duration (float) : How much time this robot motion should take

        Raises:
            FrankaArmCollisionException if a collision is detected
        '''
        if pose.from_frame != 'franka_tool' or pose.to_frame != 'world':
            raise ValueError('pose has invalid frame names! Make sure pose has from_frame=franka_tool and to_frame=world')

        tool_base_pose = tool_pose * self._tool_delta_pose.inverse()

        skill = ArmMoveToGoalWithDefaultSensorSkill()
        skill.add_initial_sensor_values(FC.EMPTY_SENSOR_VALUES)
        skill.add_feedback_controller_params(FC.DEFAULT_TORQUE_CONTROLLER_PARAMS)
        skill.add_termination_params(FC.DEFAULT_TERM_PARAMS) 

        skill.add_trajectory_params([duration] + tool_base_pose.matrix.T.flatten().tolist())
        goal = skill.create_goal()
        
        self._send_goal(goal, cb=lambda x: skill.feedback_callback(x))

    def goto_pose_delta(self, delta_tool_pose, duration=3):
        '''Commands Arm to the given delta pose via linear interpolation

        Args:
            delta_tool_pose (RigidTransform) : Delta pose in tool frame
            duration (float) : How much time this robot motion should take

        Raises:
            FrankaArmCollisionException if a collision is detected
        '''
        if delta_tool_pose.from_frame != 'franka_tool' or delta_tool_pose.to_frame != 'franka_tool':
            raise ValueError('delta_pose has invalid frame names! Make sure delta_pose has from_frame=franka_tool and to_frame=franka_tool')

        delta_tool_base_pose = self._tool_delta_pose * delta_tool_pose * self._tool_delta_pose.inverse()

        skill = ArmRelativeMotionWithDefaultSensorSkill()
        skill.add_initial_sensor_values(FC.EMPTY_SENSOR_VALUES)
        skill.add_feedback_controller_params(FC.DEFAULT_TORQUE_CONTROLLER_PARAMS) 
        skill.add_termination_params(FC.DEFAULT_TERM_PARAMS)

        skill.add_trajectory_params([duration] + delta_tool_base_pose.translation.tolist() + delta_tool_base_pose.quaternion.tolist())
        goal = skill.create_goal()
        
        self._send_goal(goal, cb=lambda x: skill.feedback_callback(x))
        
    def goto_joints(self, joints, duration=3):
        '''Commands Arm to the given joint configuration

        Args:
            joints (list): A list of 7 numbers that correspond to joint angles in radians
            duration (float) : How much time this robot motion should take       

        Raises:
            ValueError: If is_joints_reachable(joints) returns False
            FrankaArmCollisionException if a collision is detected
        '''
        if not self.is_joints_reachable(joints):
            raise ValueError('Joints not reachable!')

        skill = JointPoseWithDefaultSensorSkill()
        skill.add_initial_sensor_values(FC.EMPTY_SENSOR_VALUES)
        skill.add_termination_params(FC.DEFAULT_TERM_PARAMS)

        skill.add_trajectory_params([duration] + np.array(joints).tolist())
        goal = skill.create_goal()
        
        self._send_goal(goal, cb=lambda x: skill.feedback_callback(x))
    
    def apply_joint_torques(self, torques, duration):
        '''Commands Arm to apply given joint torques for duration seconds

        Args:
            torques (list): A list of 7 numbers that correspond to torques in Nm
            duration (float): A float in the unit of seconds

        Raises:
            FrankaArmCollisionException if a collision is detected
        '''
        pass

    def apply_effector_forces_torques(self, run_duration, acc_duration, forces=None, torques=None):
        '''Applies the given end-effector forces and torques in N and Nm

        Args:
            run_duration (float): A float in the unit of seconds
            acc_duration (float): A float in the unit of seconds. How long to acc/de-acc to achieve desired force. 
            forces (list): Optional (defaults to None). 
                A list of 3 numbers that correspond to end-effector forces in 3 directions
            torques (list): Optional (defaults to None).
                A list of 3 numbers that correspond to end-effector torques in 3 axes

        Raises:
            ValueError if acc_duration > 0.5*run_duration, or if forces are too large
            FrankaArmCollisionException if a collision is detected
        '''
        if acc_duration > 0.5 * run_duration:
            raise ValueError('acc_duration must be smaller than half of run_duration!')
            
        forces = [0, 0, 0] if forces is None else np.array(forces).tolist()
        torques = [0, 0, 0] if torques is None else np.array(torques).tolist()

        if np.linalg.norm(forces) * run_duration > FC.MAX_LIN_MOMENTUM:
            raise ValueError('Linear momentum magnitude exceeds safety threshold of {}'.format(FC.MAX_LIN_MOMENTUM))
        if np.linalg.norm(torques) * run_duration > FC.MAX_ANG_MOMENTUM:
            raise ValueError('Angular momentum magnitude exceeds safety threshold of {}'.format(FC.MAX_ANG_MOMENTUM))

        skill = ForceTorqueSkill()
        skill.add_initial_sensor_values(FC.EMPTY_SENSOR_VALUES)
        skill.add_termination_params([0])

        skill.add_trajectory_params([0, run_duration, acc_duration] + forces + torques)
        goal = skill.create_goal()
        
        self._send_goal(goal, cb=lambda x: skill.feedback_callback(x))

    def goto_gripper(self, width, speed=0.04, force=None):
        '''Commands gripper to goto a certain width, applying up to the given (default is max) force if needed

        Args:
            width (float): A float in the unit of meters
            speed (float): Gripper operation speed in meters per sec
            force (float): Max gripper force to apply in N. Default to None, which gives acceptable force

        Raises:
            ValueError: If width is less than 0 or greater than TODO(jacky) the maximum gripper opening
        '''
        skill = GripperWithDefaultSensorSkill()
        skill.add_initial_sensor_values(FC.EMPTY_SENSOR_VALUES)

        # TODO(jacky): why is wait time needed?
        if force is not None:
            skill.add_trajectory_params([width, speed, force, FC.GRIPPER_WAIT_TIME])  # Gripper Width, Gripper Speed, Wait Time
        else:
            skill.add_trajectory_params([width, speed, FC.GRIPPER_WAIT_TIME])  # Gripper Width, Gripper Speed, Wait Time
            
        goal = skill.create_goal()

        self._send_goal(goal, cb=lambda x: skill.feedback_callback(x))

    def open_gripper(self):
        '''Opens gripper to maximum width
        '''
        self.goto_gripper(FC.GRIPPER_WIDTH_MAX)

    def close_gripper(self, grasp=True):
        '''Closes the gripper as much as possible
        '''
        self.goto_gripper(FC.GRIPPER_WIDTH_MIN, force=FC.GRIPPER_MAX_FORCE if grasp else None)

    '''
    Reads
    '''
    
    def get_robot_state(self):
        '''
        Returns:
            dict of full robot state data
        '''
        return self._sub.get_data()

    def get_pose(self):
        '''
        Returns:
            pose (RigidTransform) of the current end-effector
        '''
        tool_base_pose = self._sub.get_pose()

        tool_pose = tool_base_pose * self._tool_delta_pose

        return tool_pose

    def get_joints(self):
        '''
        Returns:
            ndarray of shape (7,) of joint angles in radians
        '''
        return self._sub.get_joints()

    def get_joint_torques(self):
        '''
        Returns:
            ndarray of shape (7,) of joint torques in Nm
        '''
        return self._sub.get_joint_torques()

    def get_joint_velocities(self):
        '''
        Returns:
            ndarray of shape (7,) of joint velocities in rads/s
        '''
        return self._sub.get_joint_velocities()

    def get_gripper_width(self):
        '''
        Returns:
            float of gripper width in meters
        '''
        return self._sub.get_gripper_width()

    def get_gripper_is_grasped(self):
        '''
        Returns:
            True if gripper is grasping something. False otherwise
        '''
        return self._sub.get_gripper_is_grasped()

    def get_speed(self, speed):
        '''
        Returns:
            float of current target speed parameter
        '''
        pass

    def get_tool_base_pose(self):
        '''
        Returns:
            RigidTransform of current tool base pose
        '''
        return self._tool_delta_pose.copy()

    '''
    Sets
    '''
    
    def set_tool_delta_pose(self, tool_delta_pose):
        '''Sets the tool pose relative to the end-effector pose

        Args:
            tool_delta_pose (RigidTransform)
        '''
        if tool_delta_pose.from_frame != 'franka_tool' or tool_delta_pose.to_frame != 'franka_tool_base':
            raise ValueError('tool_delta_pose has invalid frame names! Make sure the has from_frame=franka_tool, and to_frame=franka_tool_base')

        self._tool_delta_pose = tool_delta_pose.copy()

    def set_speed(self, speed):
        '''Sets current target speed parameter
        
        Args:
            speed (float)
        '''
        pass

    '''
    Misc
    '''
    def reset_joints(self):
        '''Commands Arm to goto hardcoded home joint configuration

        Raises:
            FrankaArmCollisionException if a collision is detected
        '''
        self.goto_joints(FC.HOME_JOINTS, duration=5)

    def is_joints_reachable(self, joints):
        '''
        Returns:
            True if all joints within joint limits
        '''
        for i, val in enumerate(joints):
            if val <= FC.JOINT_LIMITS_MIN[i] or val >= FC.JOINT_LIMITS_MAX[i]:
                return False

        return True
