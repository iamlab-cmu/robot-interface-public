import roslib
roslib.load_manifest('franka_action_lib')
import rospy
import actionlib
from franka_action_lib.msg import ExecuteSkillAction

from skill_list import *
from exceptions import *

class FrankaArm:

    def __init__(self, rosnode_name='franka_arm_client'):
        rospy.init_node(rosnode_name)
        self._client = actionlib.SimpleActionClient('/execute_skill_action_server_node/execute_skill', ExecuteSkillAction)
        self._client.wait_for_server()

    def _send_goal(self, goal, cb):
        '''
        Raises:
            FrankaArmCommException if a timeout is reached
        '''
        # TODO(jacky): institute a timeout check
        self._client.send_goal(goal, feedback_cb=cb)
        done = self._client.wait_for_result(rospy.Duration.from_sec(5.0))

        while not rospy.is_shutdown() and done != True:
            done = self._client.wait_for_result(rospy.Duration.from_sec(5.0))

        return self._client.get_result()

    '''
    Controls
    '''

    def goto_pose(self, pose):
        '''Commands Arm to the given pose via linear interpolation

        Args:
            pose (RigidTransform)

        Raises:
            FrankaArmCollisionException if a collision is detected
        '''
        skill = ArmMoveToGoalWithDefaultSensorSkill()
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
        skill.add_feedback_controller_params([600, 50]) # translational stiffness, rotational stiffness
        skill.add_termination_params([1.0]) # buffer time

        skill.add_trajectory_params([3] + pose.matrix.T.flatten().tolist())
        goal = skill.create_goal()
        
        retval = self._send_goal(goal, cb=lambda x: skill.feedback_callback(x))
        # TODO(jacky): raise appropriate exceptions depending on retval

    def goto_pose_delta(self, pose):
        '''Commands Arm to the given delta pose via linear interpolation

        Args:
            pose (RigidTransform)

        Raises:
            FrankaArmCollisionException if a collision is detected
        '''
        skill = ArmRelativeMotionWithDefaultSensorSkill()
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
        skill.add_feedback_controller_params([600, 50]) # translational stiffness, rotational stiffness
        skill.add_termination_params([1.0]) # buffer time

        skill.add_trajectory_params([3] + pose.translation.tolist() + pose.quaternion.tolist())
        goal = skill.create_goal()
        
        retval = self._send_goal(goal, cb=lambda x: skill.feedback_callback(x))
        # TODO(jacky): raise appropriate exceptions depending on retval
        
    def goto_joints(self, joints):
        '''Commands Arm to the given joint configuration

        Args:
            joints (list): A list of 7 numbers that correspond to joint angles in radians

        Raises:
            ValueError: If is_joints_reachable(joints) returns False
            FrankaArmCollisionException if a collision is detected
        '''
        pass
    
    def apply_joint_torques(self, torques, duration):
        '''Commands Arm to apply given joint torques for duration seconds

        Args:
            torques (list): A list of 7 numbers that correspond to torques in Nm
            duration (float): A float in the unit of seconds

        Raises:
            FrankaArmCollisionException if a collision is detected
        '''
        pass

    def apply_effector_force(self, force, duration):
        '''Commands Arm to apply given force at the end-effector for duration seconds

        Args:
            force (list): A list of 3 numbers that correspond to end-effector forces in 3 directions
            duration (float): A float in the unit of seconds

        Raises:
            FrankaArmCollisionException if a collision is detected
        '''
        pass

    def goto_gripper_width(self, width):
        '''Commands gripper to goto a certain width, applying force if needed

        Args:
            width (float): A float in the unit of meters

        Raises:
            ValueError: If width is less than 0 or greater than TODO(jacky) the maximum gripper opening
        '''
        pass

    def goto_gripper_force(self, force):
        '''Commands gripper to close until it reaches the given force

        Args:
            force (float): A float in the unit of N
        '''
        pass

    def open_gripper(self):
        '''Opens gripper to maximum width
        '''
        pass

    '''
    Reads
    '''
    
    def get_pose(self):
        '''
        Returns:
            pose (RigidTransform) of the current end-effector

        Raises:
            FrankaArmCommException if a timeout is reached
        '''
        pass

    def get_joints(self):
        '''
        Returns:
            ndarray of shape (7,) of joint angles in radians

        Raises:
            FrankaArmCommException if a timeout is reached
        '''
        pass

    def get_torques(self):
        '''
        Returns:
            ndarray of shape (7,) of joint torques in Nm

        Raises:
            FrankaArmCommException if a timeout is reached
        '''
        pass

    def get_gripper_width(self):
        '''
        Returns:
            float of gripper width in meters

        Raises:
            FrankaArmCommException if a timeout is reached
        '''
        pass

    def get_gripper_force(self):
        '''
        Returns:
            float of gripper force in N

        Raises:
            FrankaArmCommException if a timeout is reached
        '''
        pass

    def get_speed(self, speed):
        '''
        Returns:
            float of current target speed parameter
        '''
        pass

    '''
    Sets
    '''
    
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
        pass

    def is_joints_reachable(self, joints):
        '''
        Returns:
            True if all joints within joint limits
        '''
        pass
